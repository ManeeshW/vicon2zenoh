#include "vicon2pose.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

vicon2pose::vicon2pose()
    : last_collect_time(std::chrono::steady_clock::now()),
      fast_last_collect_time(std::chrono::steady_clock::now()),
      rng(std::random_device{}()), dist(0.0, 1.0) {
    R_sv << 0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0;
    R_off << -1.0, 0.0, 0.0,
              0.0, -1.0, 0.0,
              0.0,  0.0, 1.0;
    R_nv << -1.0, 0.0,  0.0,
             0.0, 1.0,  0.0,
             0.0, 0.0, -1.0;
    R_im << 0.0, 1.0,  0.0,
            1.0, 0.0,  0.0,
            0.0, 0.0, -1.0;
}

vicon2pose::~vicon2pose() {
    close();
}

// Strip trailing whitespace and inline # comments from a value string
static std::string trim_value(const std::string& s) {
    auto cpos = s.find('#');
    std::string r = (cpos != std::string::npos) ? s.substr(0, cpos) : s;
    auto e = r.find_last_not_of(" \t\r\n");
    return (e == std::string::npos) ? "" : r.substr(0, e + 1);
}

void vicon2pose::load_config(const std::string& config_file, const std::string& section) {
    name = section;
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "[" << section << "] Could not open config file: " << config_file << std::endl;
        return;
    }

    bool in_section = false;
    std::string line;
    while (std::getline(file, line)) {
        // Strip leading whitespace
        auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);
        if (line[0] == '#') continue;

        // Section header
        if (line[0] == '[') {
            auto end = line.find(']');
            if (end != std::string::npos) {
                std::string sec = line.substr(1, end - 1);
                auto s0 = sec.find_first_not_of(" \t");
                auto s1 = sec.find_last_not_of(" \t");
                sec = (s0 == std::string::npos) ? "" : sec.substr(s0, s1 - s0 + 1);
                in_section = (sec == section);
            }
            continue;
        }
        if (!in_section) continue;

        // Find separator (: or =)
        auto sep = line.find_first_of(":=");
        if (sep == std::string::npos) continue;
        std::string key = line.substr(0, sep);
        std::string val = line.substr(sep + 1);

        // Trim key
        auto k0 = key.find_first_not_of(" \t");
        auto k1 = key.find_last_not_of(" \t");
        if (k0 == std::string::npos) continue;
        key = key.substr(k0, k1 - k0 + 1);

        // Trim value (strip leading whitespace + inline comment)
        auto v0 = val.find_first_not_of(" \t");
        val = (v0 == std::string::npos) ? "" : val.substr(v0);
        val = trim_value(val);

        if (key == "Object") {
            object_name = val;
        } else if (key == "Port") {
            try { port = std::stoi(val); } catch (...) { port = 3883; }
        } else if (key == "on") {
            on = (val == "true" || val == "1");
        } else if (key == "zenoh_key") {
            zenoh_key = val;
        } else if (key == "latency") {
            try { latency = std::stod(val); } catch (...) { latency = 0.0; }
        } else if (key == "frequency") {
            try {
                frequency = std::stod(val);
                dt_desired = 1.0 / frequency;
            } catch (...) { frequency = 5.0; dt_desired = 0.2; }
        } else if (key == "std_x") {
            try { std_x = std::stod(val); } catch (...) { std_x = 0.0; }
        } else if (key == "std_R") {
            try { std_R = std::stod(val); } catch (...) { std_R = 0.0; }
        } else if (key == "noise_x_enabled") {
            noise_x_enabled = (val == "true" || val == "1");
        } else if (key == "noise_R_enabled") {
            noise_R_enabled = (val == "true" || val == "1");
        } else if (key == "position_only") {
            position_only = (val == "true" || val == "1");
        } else if (key == "fast_key") {
            fast_key = val;
        } else if (key == "fast_enable") {
            fast_enable = (val == "true" || val == "1");
        } else if (key == "fast_frequency") {
            try {
                double ff = std::stod(val);
                if (ff > 0.0) fast_dt = 1.0 / ff;
            } catch (...) {}
        } else if (key == "fast_legacy_frame") {
            fast_legacy_frame = (val == "true" || val == "1");
        }
    }
    file.close();

    std::cout << "[" << section << "] Config: object=" << object_name << ":" << port
              << " zenoh_key=" << zenoh_key
              << " on=" << on << " frequency=" << frequency << "Hz"
              << " latency=" << latency << "s"
              << " fast=" << (fast_enable ? fast_key : std::string("off"))
              << (fast_enable && fast_legacy_frame ? " [fast:legacy_frame]" : "") << std::endl;
}

void vicon2pose::open() {
    std::string vrpn_object = object_name + ":" + std::to_string(port);
    vicon_instance.open(vrpn_object);
    zenoh::ZResult* err = nullptr;
    try {
        auto config = zenoh::Config::create_default();
        session = zenoh::Session(std::move(config), zenoh::Session::SessionOptions::create_default(), err);
        publisher = session->declare_publisher(zenoh_key, zenoh::Session::PublisherOptions::create_default(), err);
        std::cout << "VICON2POSE: publisher on " << zenoh_key << std::endl;
        if (fast_enable) {
            fast_publisher = session->declare_publisher(fast_key, zenoh::Session::PublisherOptions::create_default(), err);
            std::cout << "VICON2POSE: fast publisher on " << fast_key
                      << " (" << static_cast<int>(1.0 / fast_dt) << " Hz)" << std::endl;
        }
        on = true;
    } catch (const std::exception& e) {
        std::cerr << "VICON2POSE: Failed to open Zenoh - " << e.what() << std::endl;
        on = false;
    }
}

void vicon2pose::loop() {
    if (!on) return;

    auto now = std::chrono::steady_clock::now();

    // Fast path: independent rate, no noise, no latency; also updates x_clean_latest
    if (fast_enable && fast_publisher &&
        std::chrono::duration<double>(now - fast_last_collect_time).count() >= fast_dt) {
        auto [x_vf, R_vmf] = vicon_instance.loop();

        // NED/IMU-frame pose: x_n = R_nv * x_v, R_ni = R_nv * R_vm * R_im.
        // Always computed; feeds x_clean_latest (used by RelativePose) regardless
        // of which frame is actually published on fast_key below.
        Eigen::Vector3d x_ned = R_nv * x_vf;
        Eigen::Matrix3d R_ned = R_nv * R_vmf * R_im;

        double tsf = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // Expose clean NED pose for RelativePose (updated at fast rate when enabled)
        x_clean_latest = x_ned;
        R_clean_latest = R_ned;
        timestamp_clean_latest = tsf;
        has_data = true;

        // Published fast pose: NED by default; legacy frame if fast_legacy_frame is set
        // (used to keep fdcl/pose_sync_fast unchanged for flying the drone).
        Eigen::Vector3d xf;
        Eigen::Matrix3d Rf;
        if (fast_legacy_frame) {
            xf = R_sv * x_vf;
            xf(0) = -xf(0); xf(1) = -xf(1);
            Rf = R_off * R_sv * R_vmf;
        } else {
            xf = x_ned;
            Rf = R_ned;
        }

        // Display always shows the NED pose (matches zenoh_key/x_clean_latest semantics),
        // independent of what fast_legacy_frame publishes on fast_key.
        if (display_snap) display_snap->update(x_ned, R_ned);

        std::vector<std::vector<double>> pose_fast(3, std::vector<double>(4));
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) pose_fast[i][j] = Rf(i, j);
            pose_fast[i][3] = xf(i);
        }
        nlohmann::json jf;
        jf["image_taken_time"] = tsf;
        jf["pose"]             = pose_fast;
        jf["noise_x"]          = {0.0, 0.0, 0.0};
        jf["noise_angles"]     = {0.0, 0.0, 0.0};
        try { fast_publisher->put(jf.dump()); }
        catch (const std::exception& e) {
            std::cerr << "VICON2POSE: [fast] publish error: " << e.what() << std::endl;
        }
        fast_last_collect_time = now;
    }

    // Main path: collect at configured frequency, apply noise, buffer with latency
    if (std::chrono::duration<double>(now - last_collect_time).count() >= dt_desired) {
        auto [x_v, R_vm] = vicon_instance.loop();

        // NED/IMU-frame pose (always applied on the main path for both Base and Rover)
        Eigen::Vector3d x_raw = R_nv * x_v;
        Eigen::Matrix3d R_raw = R_nv * R_vm * R_im;

        double ts = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // Update clean pose (also updated by fast path above; this covers the no-fast-enable case)
        if (!fast_enable) {
            x_clean_latest = x_raw;
            R_clean_latest = R_raw;
            timestamp_clean_latest = ts;
            has_data = true;
        }

        PoseData data;
        data.x_pose       = x_raw;
        data.R_pose       = position_only ? -Eigen::Matrix3d::Identity() : R_raw;
        data.timestamp    = ts;
        data.collect_time = now;
        data.noise_x      = Eigen::Vector3d::Zero();
        data.noise_angles = Eigen::Vector3d::Zero();

        if (noise_x_enabled && std_x > 0.0) {
            data.noise_x = Eigen::Vector3d(std_x * dist(rng), std_x * dist(rng), std_x * dist(rng));
            data.x_pose += data.noise_x;
        }
        if (noise_R_enabled && std_R > 0.0 && !position_only) {
            data.noise_angles = Eigen::Vector3d(std_R * dist(rng), std_R * dist(rng), std_R * dist(rng));
            double cx = std::cos(data.noise_angles(0)), sx = std::sin(data.noise_angles(0));
            double cy = std::cos(data.noise_angles(1)), sy = std::sin(data.noise_angles(1));
            double cz = std::cos(data.noise_angles(2)), sz = std::sin(data.noise_angles(2));
            Eigen::Matrix3d Rx, Ry, Rz;
            Rx << 1, 0, 0,  0, cx, -sx,  0, sx, cx;
            Ry << cy, 0, sy, 0, 1, 0, -sy, 0, cy;
            Rz << cz, -sz, 0, sz, cz, 0, 0, 0, 1;
            data.R_pose = data.R_pose * Rz * Ry * Rx;
        }

        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            pose_buffer.push(data);
            x_pose_sync = data.x_pose;
            R_pose_sync = data.R_pose;
            timestamp   = data.timestamp;
        }
        data_cv.notify_all();
        last_collect_time = now;
    }

    // Drain latency buffer
    while (!pose_buffer.empty()) {
        auto& front = pose_buffer.front();
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - front.collect_time).count();
        if (elapsed < latency) break;

        std::vector<std::vector<double>> pose_data(3, std::vector<double>(4));
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) pose_data[i][j] = front.R_pose(i, j);
            pose_data[i][3] = front.x_pose(i);
        }
        nlohmann::json j;
        j["image_taken_time"] = front.timestamp;
        j["pose"]             = pose_data;
        j["noise_x"]          = {front.noise_x(0), front.noise_x(1), front.noise_x(2)};
        j["noise_angles"]     = {front.noise_angles(0), front.noise_angles(1), front.noise_angles(2)};
        try {
            publisher->put(j.dump());
            if (display_freq) display_freq->tick();
            if (!display_snap) {
                // TUI not active — fall back to console output
                std::cout << "[" << name << " -> " << zenoh_key << "]"
                          << "  pos: [" << front.x_pose(0) << ", "
                                        << front.x_pose(1) << ", "
                                        << front.x_pose(2) << "]"
                          << "  R(row0): [" << front.R_pose(0,0) << ", "
                                            << front.R_pose(0,1) << ", "
                                            << front.R_pose(0,2) << "]"
                          << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "VICON2POSE: Publish error - " << e.what() << std::endl;
        }
        pose_buffer.pop();
    }
}

void vicon2pose::close() {
    on = false;
    vicon_instance.close();
    publisher.reset();
    fast_publisher.reset();
    if (session) {
        session->close(zenoh::Session::SessionCloseOptions::create_default(), nullptr);
        session.reset();
    }
    std::lock_guard<std::mutex> lock(pose_mutex);
    while (!pose_buffer.empty()) pose_buffer.pop();
    std::cout << "VICON2POSE: Closed (" << object_name << ")" << std::endl;
}
