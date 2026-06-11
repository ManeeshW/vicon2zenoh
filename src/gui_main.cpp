/// gui_main.cpp — Zenoh subscriber GUI entry point for vicon2zenoh
///
/// Reads config.cfg, subscribes to all configured Zenoh topics, parses
/// incoming JSON payloads and feeds them to ViconGUI at 30 Hz via postFrame().

#include "vicon_gui.hpp"

#include <zenoh.hxx>
#include <nlohmann/json.hpp>

#include <QApplication>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cstring>

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Config parsing helpers (mirrors vicon2pose.cpp style)
// ─────────────────────────────────────────────────────────────────────────────
static std::string cfg_trim(const std::string& s) {
    const char* ws = " \t\"";
    size_t a = s.find_first_not_of(ws);
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(ws);
    return s.substr(a, b - a + 1);
}

static std::vector<std::string> cfg_split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ','))
        out.push_back(cfg_trim(tok));
    return out;
}

struct GuiConfig {
    std::vector<std::string> object_keys;
    std::string              pose_sync_key          = "fdcl/pose_sync";
    std::string              rel_gt_state_key       = "fdcl/rel_gt_state";
    bool                     gui_enable             = true;
    bool                     rel_pose_transform_enable = false;
};

static GuiConfig load_config(const std::string& path) {
    GuiConfig cfg;

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[gui_main] Config not found: " << path
                  << ", using defaults." << std::endl;
        return cfg;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Strip leading whitespace and skip comments
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos || line[start] == '#') continue;
        line = line.substr(start);

        auto val_of = [&](const std::string& prefix) -> std::string {
            if (line.size() > prefix.size() &&
                line.substr(0, prefix.size()) == prefix)
                return cfg_trim(line.substr(prefix.size()));
            return "";
        };

        std::string v;
        if (!(v = val_of("object_keys:")).empty()) {
            cfg.object_keys = cfg_split_csv(v);
        } else if (!(v = val_of("pose_sync_key:")).empty()) {
            cfg.pose_sync_key = v;
        } else if (!(v = val_of("rel_gt_state_key:")).empty()) {
            cfg.rel_gt_state_key = v;
        } else if (!(v = val_of("gui_enable:")).empty()) {
            cfg.gui_enable = (v == "true" || v == "1");
        } else if (!(v = val_of("rel_pose_transform_enable:")).empty()) {
            cfg.rel_pose_transform_enable = (v == "true" || v == "1");
        }
    }

    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON → TopicFrame helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Parse a pose message: {"image_taken_time": ns, "pose": [[R00..R02,x], ...]}
static bool parse_pose_msg(const json& j, TopicFrame& out) {
    if (!j.contains("pose")) return false;
    const auto& pose = j["pose"];
    if (!pose.is_array() || pose.size() < 3) return false;

    for (int row = 0; row < 3; ++row) {
        const auto& r = pose[row];
        if (!r.is_array() || r.size() < 4) return false;
        for (int col = 0; col < 3; ++col)
            out.R[row][col] = r[col].get<double>();
        out.pos[row] = r[3].get<double>();
    }

    // Timestamp: convert nanoseconds → seconds
    if (j.contains("image_taken_time") && j["image_taken_time"].is_number())
        out.ts = j["image_taken_time"].get<double>() * 1e-9;

    return true;
}

/// Parse a rel_gt_state message:
/// {"timestamp": sec, "rel_pos":[x,y,z], "rel_vel":[...], "rel_R":[[3x3]], "rel_omega":[...]}
static bool parse_rel_gt_state_msg(const json& j, TopicFrame& out) {
    if (!j.contains("rel_pos") || !j.contains("rel_R")) return false;

    // Position
    const auto& rp = j["rel_pos"];
    if (!rp.is_array() || rp.size() < 3) return false;
    for (int i = 0; i < 3; ++i) out.pos[i] = rp[i].get<double>();

    // Rotation matrix
    const auto& rR = j["rel_R"];
    if (!rR.is_array() || rR.size() < 3) return false;
    for (int row = 0; row < 3; ++row) {
        const auto& r = rR[row];
        if (!r.is_array() || r.size() < 3) return false;
        for (int col = 0; col < 3; ++col)
            out.R[row][col] = r[col].get<double>();
    }

    // Velocity
    if (j.contains("rel_vel")) {
        const auto& rv = j["rel_vel"];
        if (rv.is_array() && rv.size() >= 3) {
            for (int i = 0; i < 3; ++i) out.vel[i] = rv[i].get<double>();
            out.has_vel = true;
        }
    }

    // Angular velocity
    if (j.contains("rel_omega")) {
        const auto& ro = j["rel_omega"];
        if (ro.is_array() && ro.size() >= 3)
            for (int i = 0; i < 3; ++i) out.omega[i] = ro[i].get<double>();
    }

    // Timestamp (already in seconds)
    if (j.contains("timestamp") && j["timestamp"].is_number())
        out.ts = j["timestamp"].get<double>();

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Inverse XY-swap transform  T = [[0,1,0],[1,0,0],[0,0,1]]
// T is its own inverse (T@T = I), so undo = apply T again.
// Swaps x↔y components of pos/vel/omega; for R: swap rows 0,1 then cols 0,1.
// ─────────────────────────────────────────────────────────────────────────────
static void apply_xy_swap(TopicFrame& f) {
    std::swap(f.pos[0],   f.pos[1]);
    std::swap(f.vel[0],   f.vel[1]);
    std::swap(f.omega[0], f.omega[1]);

    double R[3][3];
    std::memcpy(R, f.R, sizeof(R));
    f.R[0][0] = R[1][1];  f.R[0][1] = R[1][0];  f.R[0][2] = R[1][2];
    f.R[1][0] = R[0][1];  f.R[1][1] = R[0][0];  f.R[1][2] = R[0][2];
    f.R[2][0] = R[2][1];  f.R[2][1] = R[2][0];  f.R[2][2] = R[2][2];
}

// ─────────────────────────────────────────────────────────────────────────────
// Zenoh subscriber handler (struct — avoids lambda lifetime issues)
// ─────────────────────────────────────────────────────────────────────────────
struct SubHandler {
    ViconGUI*   gui                    = nullptr;
    std::string topic;
    bool        is_rel_gt              = false;
    bool        rel_pose_transform_inv = false; // undo XY-swap on rel_gt

    void operator()(zenoh::Sample& s) const {
        // Extract payload bytes as string
        std::string raw = s.get_payload().as_string();

        TopicFrame frame;
        try {
            json j = json::parse(raw);
            bool ok = false;
            if (is_rel_gt) {
                ok = parse_rel_gt_state_msg(j, frame);
            } else {
                ok = parse_pose_msg(j, frame);
            }
            if (!ok) {
                std::cerr << "[gui_main] Payload parse failed for topic: "
                          << topic << std::endl;
                return;
            }
        } catch (const json::exception& e) {
            std::cerr << "[gui_main] JSON parse error on topic " << topic
                      << ": " << e.what() << std::endl;
            return;
        }

        // Undo publisher XY-swap so GUI shows data in original Vicon frame
        if (is_rel_gt && rel_pose_transform_inv)
            apply_xy_swap(frame);

        gui->postFrame(topic, frame);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // ── Load configuration ────────────────────────────────────────────────────
    const std::string config_path = "../config.cfg";
    GuiConfig cfg = load_config(config_path);

    if (!cfg.gui_enable) {
        std::cout << "[gui_main] gui_enable=false — exiting." << std::endl;
        return 0;
    }

    std::cout << "[gui_main] Starting Vicon2Zenoh GUI..." << std::endl;
    std::cout << "[gui_main] object_keys:      ";
    for (const auto& k : cfg.object_keys) std::cout << k << "  ";
    std::cout << "\n[gui_main] pose_sync_key:    " << cfg.pose_sync_key
              << "\n[gui_main] rel_gt_state_key: " << cfg.rel_gt_state_key
              << std::endl;

    // ── Qt Application ────────────────────────────────────────────────────────
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    // Dark palette applied on the QApplication level
    QPalette dark_pal;
    dark_pal.setColor(QPalette::Window,          QColor("#1e1e1e"));
    dark_pal.setColor(QPalette::WindowText,      QColor("#d4d4d4"));
    dark_pal.setColor(QPalette::Base,            QColor("#252526"));
    dark_pal.setColor(QPalette::AlternateBase,   QColor("#2d2d30"));
    dark_pal.setColor(QPalette::ToolTipBase,     QColor("#252526"));
    dark_pal.setColor(QPalette::ToolTipText,     QColor("#d4d4d4"));
    dark_pal.setColor(QPalette::Text,            QColor("#d4d4d4"));
    dark_pal.setColor(QPalette::Button,          QColor("#3c3c3c"));
    dark_pal.setColor(QPalette::ButtonText,      QColor("#d4d4d4"));
    dark_pal.setColor(QPalette::BrightText,      QColor("#ffffff"));
    dark_pal.setColor(QPalette::Link,            QColor("#007acc"));
    dark_pal.setColor(QPalette::Highlight,       QColor("#094771"));
    dark_pal.setColor(QPalette::HighlightedText, QColor("#ffffff"));
    app.setPalette(dark_pal);

    // ── ViconGUI window ───────────────────────────────────────────────────────
    ViconGUI gui;
    gui.setTopics(cfg.object_keys, cfg.pose_sync_key, cfg.rel_gt_state_key);
    gui.show();

    // ── Zenoh session ─────────────────────────────────────────────────────────
    auto z_config = zenoh::Config::create_default();
    zenoh::Session session(std::move(z_config),
                           zenoh::Session::SessionOptions::create_default(),
                           nullptr);

    // ── Declare subscribers ───────────────────────────────────────────────────
    // Subscribers must be kept alive for the duration of the session.
    // We store them in vectors to extend their lifetime.

    std::vector<zenoh::Subscriber<void>> subscribers;
    subscribers.reserve(cfg.object_keys.size() + 2);

    auto make_subscriber = [&](const std::string& key, bool is_rel_gt) {
        SubHandler handler;
        handler.gui                    = &gui;
        handler.topic                  = key;
        handler.is_rel_gt              = is_rel_gt;
        handler.rel_pose_transform_inv = is_rel_gt && cfg.rel_pose_transform_enable;

        std::cout << "[gui_main] Subscribing to: " << key << std::endl;
        subscribers.push_back(
            session.declare_subscriber(
                zenoh::KeyExpr(key),
                [h = std::move(handler)](zenoh::Sample& s) mutable { h(s); },
                []() {}));
    };

    // Object-key subscribers (pose format)
    for (const auto& key : cfg.object_keys)
        make_subscriber(key, false);

    // Pose-sync subscriber
    if (!cfg.pose_sync_key.empty())
        make_subscriber(cfg.pose_sync_key, false);

    // Rel-gt-state subscriber
    if (!cfg.rel_gt_state_key.empty())
        make_subscriber(cfg.rel_gt_state_key, true);

    std::cout << "[gui_main] rel_pose_transform_enable: "
              << (cfg.rel_pose_transform_enable ? "true (inverse XY-swap applied)" : "false")
              << std::endl;
    std::cout << "[gui_main] " << subscribers.size()
              << " subscriber(s) active. Showing window..." << std::endl;

    // ── Event loop ────────────────────────────────────────────────────────────
    int ret = app.exec();

    // ── Cleanup ───────────────────────────────────────────────────────────────
    subscribers.clear();
    session.close(zenoh::Session::SessionCloseOptions::create_default(), nullptr);

    std::cout << "[gui_main] Closed." << std::endl;
    return ret;
}
