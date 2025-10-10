#include "vicon2gt.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

vicon2gt::vicon2gt() : last_collect_time(std::chrono::steady_clock::now()) {
    // Initialize transformation matrix R_sv
    R_sv << 0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
    // Initialize additional transformation matrix R0
    R0 << 0.0, 1.0, 0.0,
          1.0, 0.0, 0.0,
          0.0, 0.0, -1.0;
    load_config("../config_gt.cfg");
}

vicon2gt::~vicon2gt() {
    close();
}

void vicon2gt::load_config(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "VICON2GT: Could not open config file: " << config_file << ", using defaults" << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        if (line.empty()) continue;
        if (line.find("zenoh_key:") != std::string::npos) {
            zenoh_key = line.substr(line.find("zenoh_key:") + 10);
            zenoh_key.erase(0, zenoh_key.find_first_not_of(" \t"));
            zenoh_key.erase(zenoh_key.find_last_not_of(" \t") + 1);
        } else if (line.find("latency:") != std::string::npos) {
            try {
                latency = std::stod(line.substr(line.find("latency:") + 8));
            } catch (...) {
                std::cerr << "VICON2GT: Invalid latency value, using default: 0.0" << std::endl;
                latency = 0.0;
            }
        } else if (line.find("frequency:") != std::string::npos) {
            try {
                frequency = std::stod(line.substr(line.find("frequency:") + 10));
                dt_desired = 1.0 / frequency;
            } catch (...) {
                std::cerr << "VICON2GT: Invalid frequency value, using default: 200.0" << std::endl;
                frequency = 200.0;
                dt_desired = 0.005;
            }
        } else if (line.find("alpha:") != std::string::npos) {
            try {
                alpha = std::stod(line.substr(line.find("alpha:") + 6));
            } catch (...) {
                std::cerr << "VICON2GT: Invalid alpha value, using default: 0.1" << std::endl;
                alpha = 0.1;
            }
        }
    }
    file.close();
    std::cout << "VICON2GT: Loaded config - zenoh_key: " << zenoh_key
              << ", latency: " << latency << "s, frequency: " << frequency
              << "Hz, dt_desired: " << dt_desired << "s, alpha: " << alpha << std::endl;
}

void vicon2gt::open() {
    vicon_instance.open();
    // Initialize Zenoh session and publisher
    zenoh::ZResult* err = nullptr;
    try {
        auto config = zenoh::Config::create_default();
        session = zenoh::Session(std::move(config), zenoh::Session::SessionOptions::create_default(), err);
        publisher = session->declare_publisher(zenoh_key, zenoh::Session::PublisherOptions::create_default(), err);
        std::cout << "VICON2GT: Zenoh publisher declared on " << zenoh_key << std::endl;
        on = true;
    } catch (const std::exception& e) {
        std::cerr << "VICON2GT: Failed to open Zenoh session - " << e.what() << std::endl;
        on = false;
    }
}

void vicon2gt::loop() {
    if (!on) return;

    // Collect data at the specified frequency
    auto now = std::chrono::steady_clock::now();
    if (now - last_collect_time >= std::chrono::milliseconds(static_cast<int>(dt_desired * 1000))) {
        auto [x_v, R_vm] = vicon_instance.loop();
        std::lock_guard<std::mutex> lock(gt_mutex);
        GtData data;
        R_off << -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, 1.0;
        data.x_pose = R_sv * x_v;
        data.x_pose(0) = -data.x_pose(0);
        data.x_pose(1) = -data.x_pose(1);
        data.R_pose = R_off * R_sv * R_vm;
        data.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // seconds
        data.collect_time = now;

        // Compute velocity with lowpass filter
        if (first_measurement) {
            data.velocity = Eigen::Vector3d::Zero();
            first_measurement = false;
        } else {
            double dt = data.timestamp - prev_timestamp;
            if (dt > 0.0) {
                Eigen::Vector3d raw_vel = (data.x_pose - prev_x_pose) / dt;
                data.velocity = alpha * raw_vel + (1.0 - alpha) * prev_velocity;
            } else {
                data.velocity = prev_velocity;
            }
        }
        prev_x_pose = data.x_pose;
        prev_timestamp = data.timestamp;
        prev_velocity = data.velocity;

        // Apply additional transformation R0 before sending
        data.x_pose = R0 * data.x_pose;
        data.R_pose = R0 * data.R_pose * R0;
        data.velocity = R0 * data.velocity;

        gt_buffer.push(data);
        x_pose_sync = data.x_pose; // Update for external access
        R_pose_sync = data.R_pose; // Update for external access
        v_pose_sync = data.velocity; // Update for external access
        timestamp = data.timestamp; // Update for external access
        last_collect_time = now;
        data_cv.notify_all();
    }

    // Publish data that has reached the latency delay
    while (!gt_buffer.empty()) {
        std::lock_guard<std::mutex> lock(gt_mutex);
        auto& data = gt_buffer.front();
        auto time_since_collect = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - data.collect_time).count() / 1000.0;
        if (time_since_collect >= latency) {
            // Prepare JSON payload
            nlohmann::json j;
            j["measured_time"] = data.timestamp;
            j["position"] = {data.x_pose(0), data.x_pose(1), data.x_pose(2)};
            j["velocity"] = {data.velocity(0), data.velocity(1), data.velocity(2)};
            nlohmann::json orientation = nlohmann::json::array();
            for (int i = 0; i < 3; ++i) {
                nlohmann::json row = {data.R_pose(i, 0), data.R_pose(i, 1), data.R_pose(i, 2)};
                orientation.push_back(row);
            }
            j["orientation"] = orientation;
            double system_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            j["system_time"] = system_time;
            // Publish via Zenoh
            try {
                publisher->put(j.dump());
                std::cout << "VICON2GT: Published - measured_time: " << data.timestamp
                          << ", system_time: " << system_time
                          << ", position: " << data.x_pose.transpose()
                          << ", velocity: " << data.velocity.transpose()
                          << ", orientation:\n" << data.R_pose << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "VICON2GT: Publish error - " << e.what() << std::endl;
            }
            gt_buffer.pop();
        } else {
            break; // Stop checking if the oldest data isn't ready yet
        }
    }

    // Sleep to prevent busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void vicon2gt::close() {
    on = false;
    vicon_instance.close();
    if (publisher) {
        publisher.reset();
    }
    if (session) {
        session->close(zenoh::Session::SessionCloseOptions::create_default(), nullptr);
        session.reset();
    }
    // Clear buffer
    std::lock_guard<std::mutex> lock(gt_mutex);
    while (!gt_buffer.empty()) {
        gt_buffer.pop();
    }
    std::cout << "VICON2GT: Closed" << std::endl;
}