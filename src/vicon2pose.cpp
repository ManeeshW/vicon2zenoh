#include "vicon2pose.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

vicon2pose::vicon2pose() : last_collect_time(std::chrono::steady_clock::now()), rng(std::random_device{}()), dist(0.0, 1.0) {
    // Initialize transformation matrix R_sv
    R_sv << 0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
    load_config("../config.cfg");
}

vicon2pose::~vicon2pose() {
    close();
}

void vicon2pose::load_config(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "VICON2POSE: Could not open config file: " << config_file << ", using defaults" << std::endl;
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
                std::cerr << "VICON2POSE: Invalid latency value, using default: 0.5" << std::endl;
                latency = 0.5;
            }
        } else if (line.find("frequency:") != std::string::npos) {
            try {
                frequency = std::stod(line.substr(line.find("frequency:") + 10));
                dt_desired = 1.0 / frequency;
            } catch (...) {
                std::cerr << "VICON2POSE: Invalid frequency value, using default: 5.0" << std::endl;
                frequency = 5.0;
                dt_desired = 0.2;
            }
        } else if (line.find("std_x:") != std::string::npos) {
            try {
                std_x = std::stod(line.substr(line.find("std_x:") + 6));
            } catch (...) {
                std::cerr << "VICON2POSE: Invalid std_x value, using default: 0.0" << std::endl;
                std_x = 0.0;
            }
        } else if (line.find("std_R:") != std::string::npos) {
            try {
                std_R = std::stod(line.substr(line.find("std_R:") + 6));
            } catch (...) {
                std::cerr << "VICON2POSE: Invalid std_R value, using default: 0.0" << std::endl;
                std_R = 0.0;
            }
        } else if (line.find("noise_x_enabled:") != std::string::npos) {
            std::string value = line.substr(line.find("noise_x_enabled:") + 16);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            noise_x_enabled = (value == "true" || value == "1");
        } else if (line.find("noise_R_enabled:") != std::string::npos) {
            std::string value = line.substr(line.find("noise_R_enabled:") + 16);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            noise_R_enabled = (value == "true" || value == "1");
        }
    }
    file.close();
    std::cout << "VICON2POSE: Loaded config - zenoh_key: " << zenoh_key
              << ", latency: " << latency << "s, frequency: " << frequency
              << "Hz, dt_desired: " << dt_desired << "s, std_x: " << std_x
              << ", std_R: " << std_R << ", noise_x_enabled: " << noise_x_enabled
              << ", noise_R_enabled: " << noise_R_enabled << std::endl;
}

void vicon2pose::open() {
    vicon_instance.open();
    // Initialize Zenoh session and publisher
    zenoh::ZResult* err = nullptr;
    try {
        auto config = zenoh::Config::create_default();
        session = zenoh::Session(std::move(config), zenoh::Session::SessionOptions::create_default(), err);
        publisher = session->declare_publisher(zenoh_key, zenoh::Session::PublisherOptions::create_default(), err);
        std::cout << "VICON2POSE: Zenoh publisher declared on " << zenoh_key << std::endl;
        on = true;
    } catch (const std::exception& e) {
        std::cerr << "VICON2POSE: Failed to open Zenoh session - " << e.what() << std::endl;
        on = false;
    }
}

void vicon2pose::loop() {
    if (!on) return;

    // Collect data at the specified frequency
    auto now = std::chrono::steady_clock::now();
    if (now - last_collect_time >= std::chrono::milliseconds(static_cast<int>(dt_desired * 1000))) {
        auto [x_v, R_vm] = vicon_instance.loop();
        std::lock_guard<std::mutex> lock(pose_mutex);
        PoseData data;
        data.x_pose = R_sv * x_v;
        data.x_pose(0) = -data.x_pose(0);
        data.x_pose(1) = -data.x_pose(1);
        data.R_pose = R_sv * R_vm;
        data.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        data.collect_time = now;
        data.noise_x = Eigen::Vector3d::Zero();
        data.noise_angles = Eigen::Vector3d::Zero();

        // Add noise to position if enabled
        if (noise_x_enabled && std_x > 0.0) {
            data.noise_x(0) = std_x * dist(rng);
            data.noise_x(1) = std_x * dist(rng);
            data.noise_x(2) = std_x * dist(rng);
            data.x_pose += data.noise_x;
            std::cout << "VICON2POSE: Applied position noise: " << data.noise_x.transpose() << std::endl;
        }

        // Add noise to rotation if enabled
        if (noise_R_enabled && std_R > 0.0) {
            // Generate small angle rotations around x, y, z axes
            data.noise_angles(0) = std_R * dist(rng);
            data.noise_angles(1) = std_R * dist(rng);
            data.noise_angles(2) = std_R * dist(rng);
            Eigen::Matrix3d Rx, Ry, Rz;
            Rx << 1.0, 0.0, 0.0,
                  0.0, std::cos(data.noise_angles(0)), -std::sin(data.noise_angles(0)),
                  0.0, std::sin(data.noise_angles(0)), std::cos(data.noise_angles(0));
            Ry << std::cos(data.noise_angles(1)), 0.0, std::sin(data.noise_angles(1)),
                  0.0, 1.0, 0.0,
                  -std::sin(data.noise_angles(1)), 0.0, std::cos(data.noise_angles(1));
            Rz << std::cos(data.noise_angles(2)), -std::sin(data.noise_angles(2)), 0.0,
                  std::sin(data.noise_angles(2)), std::cos(data.noise_angles(2)), 0.0,
                  0.0, 0.0, 1.0;
            data.R_pose = data.R_pose * Rz * Ry * Rx;
            std::cout << "VICON2POSE: Applied rotation noise angles (rad): " << data.noise_angles.transpose() << std::endl;
        }

        pose_buffer.push(data);
        x_pose_sync = data.x_pose; // Update for external access
        R_pose_sync = data.R_pose; // Update for external access
        timestamp = data.timestamp; // Update for external access
        last_collect_time = now;
        data_cv.notify_all();
    }

    // Publish data that has reached the latency delay
    while (!pose_buffer.empty()) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        auto& data = pose_buffer.front();
        auto time_since_collect = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - data.collect_time).count() / 1000.0;
        if (time_since_collect >= latency) {
            // Prepare JSON payload
            nlohmann::json j;
            j["image_taken_time"] = data.timestamp;
            std::vector<std::vector<double>> pose_data(3, std::vector<double>(4));
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    pose_data[i][j] = static_cast<double>(data.R_pose(i, j));
                }
                pose_data[i][3] = static_cast<double>(data.x_pose(i));
            }
            j["pose"] = pose_data;
            j["noise_x"] = {data.noise_x(0), data.noise_x(1), data.noise_x(2)};
            j["noise_angles"] = {data.noise_angles(0), data.noise_angles(1), data.noise_angles(2)};
            // Publish via Zenoh
            try {
                publisher->put(j.dump());
                std::cout << "VICON2POSE: Published - t: " << data.timestamp
                          << ", x_pose: " << data.x_pose.transpose()
                          << ", R_pose:\n" << data.R_pose
                          << ", noise_x: " << data.noise_x.transpose()
                          << ", noise_angles: " << data.noise_angles.transpose() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "VICON2POSE: Publish error - " << e.what() << std::endl;
            }
            pose_buffer.pop();
        } else {
            break; // Stop checking if the oldest data isn't ready yet
        }
    }

    // Sleep to prevent busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void vicon2pose::close() {
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
    std::lock_guard<std::mutex> lock(pose_mutex);
    while (!pose_buffer.empty()) {
        pose_buffer.pop();
    }
    std::cout << "VICON2POSE: Closed" << std::endl;
}