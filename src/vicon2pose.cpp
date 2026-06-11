#include "vicon2pose.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <nlohmann/json.hpp>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::string trim(const std::string& s) {
    const char* ws = " \t\"";
    size_t a = s.find_first_not_of(ws);
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(ws);
    return s.substr(a, b - a + 1);
}

static std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ','))
        out.push_back(trim(tok));
    return out;
}

// ---------------------------------------------------------------------------
// KalmanCA1D  —  constant-acceleration model, position measurement
// ---------------------------------------------------------------------------
double KalmanCA1D::update(double z, double dt, double q, double r) {
    if (!initialized) {
        x(0) = z; x(1) = 0.0; x(2) = 0.0;
        initialized = true;
        return 0.0;
    }

    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;

    Eigen::Matrix3d F;
    F << 1, dt, 0.5*dt2,
         0,  1,      dt,
         0,  0,       1;

    // Van Loan process noise (white noise on acceleration)
    Eigen::Matrix3d Q;
    Q << dt5/20.0, dt4/8.0, dt3/6.0,
         dt4/8.0,  dt3/3.0, dt2/2.0,
         dt3/6.0,  dt2/2.0,      dt;
    Q *= q;

    // Predict
    x = F * x;
    P = F * P * F.transpose() + Q;

    // Update  (H = [1, 0, 0])
    double S = P(0, 0) + r;
    Eigen::Vector3d K = P.col(0) / S;
    x += K * (z - x(0));
    P -= K * P.row(0);

    return x(1); // filtered velocity
}

// ---------------------------------------------------------------------------
// KalmanCV1D  —  constant-velocity model, omega measurement
// ---------------------------------------------------------------------------
double KalmanCV1D::update(double z, double dt, double q, double r) {
    if (!initialized) {
        x(0) = z; x(1) = 0.0;
        initialized = true;
        return z;
    }

    double dt2 = dt * dt, dt3 = dt2 * dt;

    Eigen::Matrix2d F;
    F << 1, dt,
         0,  1;

    Eigen::Matrix2d Q;
    Q << dt3/3.0, dt2/2.0,
         dt2/2.0,      dt;
    Q *= q;

    // Predict
    x = F * x;
    P = F * P * F.transpose() + Q;

    // Update  (H = [1, 0])
    double S = P(0, 0) + r;
    Eigen::Vector2d K = P.col(0) / S;
    x += K * (z - x(0));
    P -= K * P.row(0);

    return x(0); // smoothed omega
}

// ---------------------------------------------------------------------------
// ObjectTracker
// ---------------------------------------------------------------------------
ObjectTracker::ObjectTracker(const std::string& name, const std::string& key,
                              zenoh::Session& session, const TrackerConfig& cfg)
    : name(name), key(key),
      latency(cfg.latency), dt_desired(cfg.dt_desired),
      std_x(cfg.std_x), std_R(cfg.std_R),
      noise_x_enabled(cfg.noise_x_enabled), noise_R_enabled(cfg.noise_R_enabled),
      last_collect_time(std::chrono::steady_clock::now()),
      rng(std::random_device{}()), dist(0.0, 1.0) {

    Eigen::Matrix3d R_sv, R_off;
    R_sv << 0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0;
    R_off << -1.0, 0.0, 0.0,
              0.0, -1.0, 0.0,
              0.0,  0.0, 1.0;
    vicon_to_body = R_off * R_sv;

    vicon_instance.open(name);
    publisher = session.declare_publisher(
        key, zenoh::Session::PublisherOptions::create_default(), nullptr);
    std::cout << "[ObjectTracker] '" << name << "' -> '" << key << "'" << std::endl;
}

void ObjectTracker::loop() {
    updated = false;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_collect_time).count();

    if (elapsed >= dt_desired) {
        auto [x_v, R_vm] = vicon_instance.loop();

        PoseData data;
        data.x_pose = vicon_to_body * x_v;
        data.R_pose = vicon_to_body * R_vm;
        data.timestamp = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        data.collect_time = now;
        data.noise_x      = Eigen::Vector3d::Zero();
        data.noise_angles = Eigen::Vector3d::Zero();

        if (noise_x_enabled && std_x > 0.0) {
            data.noise_x(0) = std_x * dist(rng);
            data.noise_x(1) = std_x * dist(rng);
            data.noise_x(2) = std_x * dist(rng);
            data.x_pose += data.noise_x;
        }

        if (noise_R_enabled && std_R > 0.0) {
            data.noise_angles(0) = std_R * dist(rng);
            data.noise_angles(1) = std_R * dist(rng);
            data.noise_angles(2) = std_R * dist(rng);
            double cx = std::cos(data.noise_angles(0)), sx = std::sin(data.noise_angles(0));
            double cy = std::cos(data.noise_angles(1)), sy = std::sin(data.noise_angles(1));
            double cz = std::cos(data.noise_angles(2)), sz = std::sin(data.noise_angles(2));
            Eigen::Matrix3d Rx, Ry, Rz;
            Rx << 1,0,0, 0,cx,-sx, 0,sx,cx;
            Ry << cy,0,sy, 0,1,0, -sy,0,cy;
            Rz << cz,-sz,0, sz,cz,0, 0,0,1;
            data.R_pose = data.R_pose * Rz * Ry * Rx;
        }

        pose_buffer.push(data);
        x_pose    = data.x_pose;
        R_pose    = data.R_pose;
        timestamp = data.timestamp;
        has_data  = true;
        updated   = true;
        last_collect_time = now;
    }

    // Drain buffer: publish entries past the latency delay
    while (!pose_buffer.empty()) {
        auto& d = pose_buffer.front();
        double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - d.collect_time).count();
        if (age >= latency) {
            nlohmann::json j;
            j["image_taken_time"] = d.timestamp;
            std::vector<std::vector<double>> pose_data(3, std::vector<double>(4));
            for (int i = 0; i < 3; ++i) {
                for (int c = 0; c < 3; ++c) pose_data[i][c] = d.R_pose(i, c);
                pose_data[i][3] = d.x_pose(i);
            }
            j["pose"]         = pose_data;
            j["noise_x"]      = {d.noise_x(0), d.noise_x(1), d.noise_x(2)};
            j["noise_angles"] = {d.noise_angles(0), d.noise_angles(1), d.noise_angles(2)};
            try {
                publisher->put(j.dump());
                double roll  = std::atan2(d.R_pose(2,1), d.R_pose(2,2)) * 180.0 / M_PI;
                double pitch = std::asin(std::max(-1.0, std::min(1.0, -d.R_pose(2,0)))) * 180.0 / M_PI;
                double yaw   = std::atan2(d.R_pose(1,0), d.R_pose(0,0)) * 180.0 / M_PI;
                std::cout << "[" << key << "] x=" << d.x_pose.transpose()
                          << "  rpy_deg=[" << roll << ", " << pitch << ", " << yaw << "]"
                          << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[" << key << "] publish error: " << e.what() << std::endl;
            }
            pose_buffer.pop();
        } else {
            break;
        }
    }
}

void ObjectTracker::close() {
    vicon_instance.close();
    publisher.reset();
    while (!pose_buffer.empty()) pose_buffer.pop();
    std::cout << "[ObjectTracker] Closed '" << name << "'" << std::endl;
}

// ---------------------------------------------------------------------------
// vicon2pose
// ---------------------------------------------------------------------------
vicon2pose::vicon2pose() {
    object_names = {"OriginsX@192.168.10.1", "OriginsY@192.168.10.1"};
    object_keys  = {"fdcl/object1", "fdcl/object2"};
    frequency    = 200.0;
    dt_desired   = 1.0 / frequency;
    load_config("../config.cfg");
}

vicon2pose::~vicon2pose() {
    close();
}

void vicon2pose::load_config(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "[vicon2pose] Config not found: " << config_file
                  << ", using defaults" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        size_t a = line.find_first_not_of(" \t");
        if (a == std::string::npos || line[a] == '#') continue;
        line = line.substr(a);

        auto val_of = [&](const std::string& prefix) -> std::string {
            if (line.size() > prefix.size() && line.substr(0, prefix.size()) == prefix)
                return trim(line.substr(prefix.size()));
            return "";
        };

        std::string v;
        if (!(v = val_of("objects:")).empty()) {
            object_names = split_csv(v);
        } else if (!(v = val_of("object_keys:")).empty()) {
            object_keys = split_csv(v);
        } else if (!(v = val_of("pose_sync_from:")).empty()) {
            try { pose_sync_from = std::stoi(v); } catch (...) {}
        } else if (!(v = val_of("pose_sync_to:")).empty()) {
            try { pose_sync_to = std::stoi(v); } catch (...) {}
        } else if (!(v = val_of("pose_sync_key:")).empty()) {
            pose_sync_key = v;
        } else if (!(v = val_of("latency:")).empty()) {
            try { latency = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("frequency:")).empty()) {
            try { frequency = std::stod(v); dt_desired = 1.0 / frequency; } catch (...) {}
        } else if (!(v = val_of("on:")).empty()) {
            on = (v == "true" || v == "1");
        } else if (!(v = val_of("std_x:")).empty()) {
            try { std_x = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("std_R:")).empty()) {
            try { std_R = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("noise_x_enabled:")).empty()) {
            noise_x_enabled = (v == "true" || v == "1");
        } else if (!(v = val_of("noise_R_enabled:")).empty()) {
            noise_R_enabled = (v == "true" || v == "1");
        } else if (!(v = val_of("rel_gt_state_key:")).empty()) {
            rel_gt_state_key = v;
        } else if (!(v = val_of("rel_gt_state_enable:")).empty()) {
            rel_gt_state_enable = (v == "true" || v == "1");
        } else if (!(v = val_of("rel_pose_transform_enable:")).empty()) {
            rel_pose_transform_enable = (v == "true" || v == "1");
        } else if (!(v = val_of("kf_q_pos:")).empty()) {
            try { kf_q_pos = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("kf_r_pos:")).empty()) {
            try { kf_r_pos = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("kf_q_omega:")).empty()) {
            try { kf_q_omega = std::stod(v); } catch (...) {}
        } else if (!(v = val_of("kf_r_omega:")).empty()) {
            try { kf_r_omega = std::stod(v); } catch (...) {}
        } else if (line.size() > 7 && line.substr(0, 7) == "object_"
                   && std::isdigit(static_cast<unsigned char>(line[7]))) {
            // Per-object override: object_N_<setting>: <value>
            size_t ul = line.find('_', 7);
            if (ul != std::string::npos) {
                try {
                    int idx = std::stoi(line.substr(7, ul - 7));
                    auto& ov = object_overrides[idx];
                    std::string rest = line.substr(ul + 1);
                    auto pv = [&](const std::string& pfx) -> std::string {
                        if (rest.size() > pfx.size() && rest.substr(0, pfx.size()) == pfx)
                            return trim(rest.substr(pfx.size()));
                        return "";
                    };
                    std::string pval;
                    if (!(pval = pv("frequency:")).empty())
                        ov.frequency = std::stod(pval);
                    else if (!(pval = pv("latency:")).empty())
                        ov.latency = std::stod(pval);
                    else if (!(pval = pv("std_x:")).empty())
                        ov.std_x = std::stod(pval);
                    else if (!(pval = pv("std_R:")).empty())
                        ov.std_R = std::stod(pval);
                    else if (!(pval = pv("noise_x_enabled:")).empty())
                        ov.noise_x_enabled = (pval == "true" || pval == "1");
                    else if (!(pval = pv("noise_R_enabled:")).empty())
                        ov.noise_R_enabled = (pval == "true" || pval == "1");
                } catch (...) {}
            }
        }
    }
    file.close();

    std::cout << "[vicon2pose] Config: " << object_names.size() << " objects, "
              << "default f=" << frequency << "Hz, latency=" << latency << "s, "
              << "pose_sync [" << pose_sync_from << "]->[" << pose_sync_to
              << "] -> '" << pose_sync_key << "'\n"
              << "  rel_gt_state=" << (rel_gt_state_enable ? rel_gt_state_key : "disabled")
              << "  transform=" << (rel_pose_transform_enable ? "on" : "off")
              << "  KF: q_pos=" << kf_q_pos << " r_pos=" << kf_r_pos
              << " q_omega=" << kf_q_omega << " r_omega=" << kf_r_omega << std::endl;
}

void vicon2pose::open() {
    try {
        auto config = zenoh::Config::create_default();
        session = zenoh::Session(std::move(config),
                                  zenoh::Session::SessionOptions::create_default(), nullptr);
        sync_publisher = session->declare_publisher(
            pose_sync_key, zenoh::Session::PublisherOptions::create_default(), nullptr);

        if (rel_gt_state_enable) {
            rel_gt_state_pub = session->declare_publisher(
                rel_gt_state_key, zenoh::Session::PublisherOptions::create_default(), nullptr);
            std::cout << "[vicon2pose] rel_gt_state -> '" << rel_gt_state_key << "'" << std::endl;
        }

        // Auto-fill object_keys if fewer entries than object_names
        while (object_keys.size() < object_names.size())
            object_keys.push_back("fdcl/object" + std::to_string(object_keys.size() + 1));

        for (size_t i = 0; i < object_names.size(); ++i) {
            TrackerConfig cfg{latency, dt_desired, std_x, std_R, noise_x_enabled, noise_R_enabled};
            auto it = object_overrides.find(static_cast<int>(i));
            if (it != object_overrides.end()) {
                auto& ov = it->second;
                if (ov.frequency)       cfg.dt_desired      = 1.0 / *ov.frequency;
                if (ov.latency)         cfg.latency         = *ov.latency;
                if (ov.std_x)           cfg.std_x           = *ov.std_x;
                if (ov.std_R)           cfg.std_R           = *ov.std_R;
                if (ov.noise_x_enabled) cfg.noise_x_enabled = *ov.noise_x_enabled;
                if (ov.noise_R_enabled) cfg.noise_R_enabled = *ov.noise_R_enabled;
                double eff_freq = 1.0 / cfg.dt_desired;
                std::cout << "[vicon2pose] object_" << i << " overrides: f=" << eff_freq
                          << "Hz, latency=" << cfg.latency << "s" << std::endl;
            }
            trackers.push_back(std::make_unique<ObjectTracker>(
                object_names[i], object_keys[i], *session, cfg));
        }

        on = true;
        std::cout << "[vicon2pose] " << trackers.size()
                  << " trackers opened, sync -> '" << pose_sync_key << "'" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[vicon2pose] Open failed: " << e.what() << std::endl;
        on = false;
    }
}

void vicon2pose::queue_and_drain_relative_pose() {
    int fi = pose_sync_from;
    int ti = pose_sync_to;
    if (fi >= static_cast<int>(trackers.size()) ||
        ti >= static_cast<int>(trackers.size()))
        return;

    auto& from_t = *trackers[fi];
    auto& to_t   = *trackers[ti];
    if (!from_t.has_data || !to_t.has_data)
        return;

    // Queue a new entry whenever either tracker received fresh data
    if (from_t.updated || to_t.updated) {
        Eigen::Vector3d x_rel = from_t.R_pose.transpose() * (to_t.x_pose - from_t.x_pose);
        Eigen::Matrix3d R_rel = from_t.R_pose.transpose() * to_t.R_pose;
        double ts     = std::max(from_t.timestamp, to_t.timestamp);
        double ts_sec = ts / 1e9;

        // Queue for pose_sync (honours latency)
        RelPoseData rd;
        rd.x_rel        = x_rel;
        rd.R_rel        = R_rel;
        rd.timestamp    = ts;
        rd.collect_time = std::chrono::steady_clock::now();
        rel_buffer.push(rd);

        // --- rel_gt_state: zero-latency, Kalman-filtered velocity ---
        if (rel_gt_state_enable && rel_gt_state_pub) {
            if (!rel_has_prev) {
                // First sample: seed KF position states, skip publishing
                for (int ax = 0; ax < 3; ++ax)
                    kf_pos[ax].update(x_rel(ax), dt_desired, kf_q_pos, kf_r_pos);
                R_rel_prev   = R_rel;
                rel_prev_ts  = ts_sec;
                rel_has_prev = true;
            } else {
                double dt = ts_sec - rel_prev_ts;
                if (dt > 1e-6) {
                    // Kalman-filtered velocity from position measurements
                    Eigen::Vector3d v_filt;
                    for (int ax = 0; ax < 3; ++ax)
                        v_filt(ax) = kf_pos[ax].update(x_rel(ax), dt, kf_q_pos, kf_r_pos);

                    // Finite-difference angular velocity, then Kalman-smoothed
                    Eigen::Matrix3d Sk = R_rel.transpose() * ((R_rel - R_rel_prev) / dt);
                    Eigen::Vector3d omega_raw(
                        (Sk(2,1) - Sk(1,2)) / 2.0,
                        (Sk(0,2) - Sk(2,0)) / 2.0,
                        (Sk(1,0) - Sk(0,1)) / 2.0);
                    Eigen::Vector3d omega_filt;
                    for (int ax = 0; ax < 3; ++ax)
                        omega_filt(ax) = kf_omega[ax].update(omega_raw(ax), dt, kf_q_omega, kf_r_omega);

                    R_rel_prev  = R_rel;
                    rel_prev_ts = ts_sec;

                    // Apply output-frame transform: T * v, T * R * T^T (T is symmetric)
                    const Eigen::Matrix3d& T = rel_pose_transform_enable
                                               ? T_rel
                                               : Eigen::Matrix3d::Identity();
                    Eigen::Vector3d pos_out   = T * x_rel;
                    Eigen::Vector3d vel_out   = T * v_filt;
                    Eigen::Matrix3d R_out     = T * R_rel * T.transpose();
                    Eigen::Vector3d omega_out = T * omega_filt;

                    std::vector<std::vector<double>> R_mat(3, std::vector<double>(3));
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            R_mat[r][c] = R_out(r, c);

                    nlohmann::json jg;
                    jg["timestamp"] = ts_sec;
                    jg["rel_pos"]   = {pos_out(0),   pos_out(1),   pos_out(2)};
                    jg["rel_vel"]   = {vel_out(0),   vel_out(1),   vel_out(2)};
                    jg["rel_omega"] = {omega_out(0), omega_out(1), omega_out(2)};
                    jg["rel_R"]     = R_mat;
                    try {
                        rel_gt_state_pub->put(jg.dump());
                    } catch (const std::exception& e) {
                        std::cerr << "[rel_gt_state] publish error: " << e.what() << std::endl;
                    }
                }
            }
        }
    }

    // Drain buffer for pose_sync
    while (!rel_buffer.empty()) {
        auto& d = rel_buffer.front();
        double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - d.collect_time).count();
        if (age >= latency) {
            nlohmann::json j;
            j["image_taken_time"] = d.timestamp;
            std::vector<std::vector<double>> pose_data(3, std::vector<double>(4));
            for (int i = 0; i < 3; ++i) {
                for (int c = 0; c < 3; ++c) pose_data[i][c] = d.R_rel(i, c);
                pose_data[i][3] = d.x_rel(i);
            }
            j["pose"]         = pose_data;
            j["noise_x"]      = {0.0, 0.0, 0.0};
            j["noise_angles"] = {0.0, 0.0, 0.0};
            j["from_key"]     = object_keys[fi];
            j["to_key"]       = object_keys[ti];
            try {
                sync_publisher->put(j.dump());
                double roll  = std::atan2(d.R_rel(2,1), d.R_rel(2,2)) * 180.0 / M_PI;
                double pitch = std::asin(std::max(-1.0, std::min(1.0, -d.R_rel(2,0)))) * 180.0 / M_PI;
                double yaw   = std::atan2(d.R_rel(1,0), d.R_rel(0,0)) * 180.0 / M_PI;
                std::cout << "[" << pose_sync_key << "] '"
                          << object_keys[fi] << "'->'" << object_keys[ti]
                          << "' x_rel=" << d.x_rel.transpose()
                          << "  rpy_deg=[" << roll << ", " << pitch << ", " << yaw << "]"
                          << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[" << pose_sync_key << "] publish error: "
                          << e.what() << std::endl;
            }
            rel_buffer.pop();
        } else {
            break;
        }
    }
}

void vicon2pose::loop() {
    if (!on) return;
    for (auto& t : trackers) t->loop();
    queue_and_drain_relative_pose();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void vicon2pose::close() {
    on = false;
    for (auto& t : trackers) t->close();
    trackers.clear();
    while (!rel_buffer.empty()) rel_buffer.pop();
    sync_publisher.reset();
    rel_gt_state_pub.reset();
    rel_has_prev = false;
    if (session) {
        session->close(zenoh::Session::SessionCloseOptions::create_default(), nullptr);
        session.reset();
    }
    std::cout << "[vicon2pose] Closed" << std::endl;
}
