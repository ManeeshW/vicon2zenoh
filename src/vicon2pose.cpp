#include "vicon2pose.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

// ---------------------------------------------------------------------------
// KalmanCA1D  —  constant-acceleration model, position measurement
// ---------------------------------------------------------------------------
double KalmanCA1D::update(double z, double dt, double q, double r) {
    if (!initialized) {
        x(0) = z; x(1) = 0.0; x(2) = 0.0;
        initialized = true;
        return 0.0;
    }
    double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, dt5 = dt4*dt;
    Eigen::Matrix3d F;
    F << 1, dt, 0.5*dt2,
         0,  1,      dt,
         0,  0,       1;
    Eigen::Matrix3d Q;
    Q << dt5/20.0, dt4/8.0, dt3/6.0,
         dt4/8.0,  dt3/3.0, dt2/2.0,
         dt3/6.0,  dt2/2.0,      dt;
    Q *= q;
    x = F * x;
    P = F * P * F.transpose() + Q;
    double S = P(0,0) + r;
    Eigen::Vector3d K = P.col(0) / S;
    x += K * (z - x(0));
    P -= K * P.row(0);
    return x(1);
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
    double dt2 = dt*dt, dt3 = dt2*dt;
    Eigen::Matrix2d F;
    F << 1, dt,
         0,  1;
    Eigen::Matrix2d Q;
    Q << dt3/3.0, dt2/2.0,
         dt2/2.0,      dt;
    Q *= q;
    x = F * x;
    P = F * P * F.transpose() + Q;
    double S = P(0,0) + r;
    Eigen::Vector2d K = P.col(0) / S;
    x += K * (z - x(0));
    P -= K * P.row(0);
    return x(0);
}

vicon2pose::vicon2pose() : last_collect_time(std::chrono::steady_clock::now()),
                           fast_last_collect_time(std::chrono::steady_clock::now()),
                           rng(std::random_device{}()), dist(0.0, 1.0) {
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
        } else if (line.find("position_only:") != std::string::npos) {
            std::string value = line.substr(line.find("position_only:") + 14);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            position_only = (value == "true" || value == "1");
        } else if (line.find("fast_key:") != std::string::npos) {
            fast_key = line.substr(line.find("fast_key:") + 9);
            fast_key.erase(0, fast_key.find_first_not_of(" \t"));
            fast_key.erase(fast_key.find_last_not_of(" \t") + 1);
        } else if (line.find("fast_enable:") != std::string::npos) {
            std::string value = line.substr(line.find("fast_enable:") + 12);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            fast_enable = (value == "true" || value == "1");
        } else if (line.find("fast_frequency:") != std::string::npos) {
            try {
                double ff = std::stod(line.substr(line.find("fast_frequency:") + 15));
                if (ff > 0.0) fast_dt = 1.0 / ff;
            } catch (...) {}
        } else if (line.find("gt_state_key:") != std::string::npos) {
            gt_state_key = line.substr(line.find("gt_state_key:") + 13);
            gt_state_key.erase(0, gt_state_key.find_first_not_of(" \t"));
            gt_state_key.erase(gt_state_key.find_last_not_of(" \t") + 1);
        } else if (line.find("gt_state_enable:") != std::string::npos) {
            std::string value = line.substr(line.find("gt_state_enable:") + 16);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            gt_state_enable = (value == "true" || value == "1");
        } else if (line.find("gt_transform_enable:") != std::string::npos) {
            std::string value = line.substr(line.find("gt_transform_enable:") + 20);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            gt_transform_enable = (value == "true" || value == "1");
        } else if (line.find("gt_transform_matrix:") != std::string::npos) {
            std::string val = line.substr(line.find("gt_transform_matrix:") + 20);
            val.erase(0, val.find_first_not_of(" \t"));
            val.erase(val.find_last_not_of(" \t") + 1);
            std::stringstream ss(val);
            std::string tok;
            std::vector<double> vals;
            while (std::getline(ss, tok, ',')) {
                tok.erase(0, tok.find_first_not_of(" \t"));
                tok.erase(tok.find_last_not_of(" \t") + 1);
                try { vals.push_back(std::stod(tok)); } catch (...) {}
            }
            if (vals.size() == 9) {
                for (int ri = 0; ri < 3; ++ri)
                    for (int ci = 0; ci < 3; ++ci)
                        T_gt(ri, ci) = vals[ri*3 + ci];
            } else {
                std::cerr << "VICON2POSE: gt_transform_matrix needs 9 values, got " << vals.size() << std::endl;
            }
        } else if (line.find("kf_q_pos:") != std::string::npos) {
            try { kf_q_pos = std::stod(line.substr(line.find("kf_q_pos:") + 9)); } catch (...) {}
        } else if (line.find("kf_r_pos:") != std::string::npos) {
            try { kf_r_pos = std::stod(line.substr(line.find("kf_r_pos:") + 9)); } catch (...) {}
        } else if (line.find("kf_q_omega:") != std::string::npos) {
            try { kf_q_omega = std::stod(line.substr(line.find("kf_q_omega:") + 11)); } catch (...) {}
        } else if (line.find("kf_r_omega:") != std::string::npos) {
            try { kf_r_omega = std::stod(line.substr(line.find("kf_r_omega:") + 11)); } catch (...) {}
        } else if (line.find("vel_smooth_alpha:") != std::string::npos) {
            try { vel_smooth_alpha = std::stod(line.substr(line.find("vel_smooth_alpha:") + 17)); } catch (...) {}
        } else if (line.find("omega_smooth_alpha:") != std::string::npos) {
            try { omega_smooth_alpha = std::stod(line.substr(line.find("omega_smooth_alpha:") + 19)); } catch (...) {}
        }
    }
    file.close();
    std::cout << "VICON2POSE: Loaded config - zenoh_key: " << zenoh_key
              << ", latency: " << latency << "s, frequency: " << frequency
              << "Hz, dt_desired: " << dt_desired << "s, std_x: " << std_x
              << ", std_R: " << std_R << ", noise_x_enabled: " << noise_x_enabled
              << ", noise_R_enabled: " << noise_R_enabled
              << ", position_only: " << position_only
              << ", gt_state=" << (gt_state_enable ? gt_state_key : "disabled")
              << ", gt_transform=" << (gt_transform_enable ? "on" : "off")
              << ", KF: q_pos=" << kf_q_pos << " r_pos=" << kf_r_pos
              << " q_omega=" << kf_q_omega << " r_omega=" << kf_r_omega
              << ", EMA: vel_alpha=" << vel_smooth_alpha << " omega_alpha=" << omega_smooth_alpha << std::endl;
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
        if (fast_enable) {
            fast_publisher = session->declare_publisher(
                fast_key, zenoh::Session::PublisherOptions::create_default(), err);
            std::cout << "VICON2POSE: Fast publisher declared on " << fast_key
                      << " (" << static_cast<int>(1.0 / fast_dt) << " Hz, no noise, no latency)" << std::endl;
        }
        if (gt_state_enable) {
            gt_state_pub = session->declare_publisher(
                gt_state_key, zenoh::Session::PublisherOptions::create_default(), err);
            std::cout << "VICON2POSE: GT state publisher declared on " << gt_state_key << std::endl;
        }
        on = true;
    } catch (const std::exception& e) {
        std::cerr << "VICON2POSE: Failed to open Zenoh session - " << e.what() << std::endl;
        on = false;
    }
}

void vicon2pose::loop() {
    if (!on) return;

    auto now = std::chrono::steady_clock::now();

    // Fast publisher: independent rate, no noise, no latency
    if (fast_enable && fast_publisher &&
        std::chrono::duration<double>(now - fast_last_collect_time).count() >= fast_dt) {
        auto [x_vf, R_vmf] = vicon_instance.loop();
        Eigen::Matrix3d R_off_f;
        R_off_f << -1.0, 0.0, 0.0,
                    0.0, -1.0, 0.0,
                    0.0,  0.0, 1.0;
        Eigen::Vector3d xf = R_sv * x_vf;
        xf(0) = -xf(0);
        xf(1) = -xf(1);
        Eigen::Matrix3d Rf = R_off_f * R_sv * R_vmf;
        double tsf = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
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
        try {
            fast_publisher->put(jf.dump());
        } catch (const std::exception& e) {
            std::cerr << "VICON2POSE: [fast] publish error: " << e.what() << std::endl;
        }
        fast_last_collect_time = now;
    }

    // Collect data at the specified frequency
    if (now - last_collect_time >= std::chrono::milliseconds(static_cast<int>(dt_desired * 1000))) {
        auto [x_v, R_vm] = vicon_instance.loop();
        std::lock_guard<std::mutex> lock(pose_mutex);
        PoseData data;
        R_off << -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, 1.0;
        data.x_pose = R_sv * x_v;
        data.x_pose(0) = -data.x_pose(0);
        data.x_pose(1) = -data.x_pose(1);
        data.R_pose = R_off * R_sv * R_vm;   // real attitude by default

        // ============== NEW: POSITION_ONLY MODE ==============
        if (position_only) {
            data.R_pose = -Eigen::Matrix3d::Identity();  // fake negative identity
            data.noise_angles = Eigen::Vector3d::Zero();
        }
        // =====================================================

        // Save clean (pre-noise) pose for GT publishing
        const Eigen::Vector3d x_clean = data.x_pose;
        const Eigen::Matrix3d R_clean = data.R_pose;

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

        // Add noise to rotation if enabled (disabled in position_only mode)
        if (noise_R_enabled && std_R > 0.0 && !position_only) {
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
        x_pose_sync = data.x_pose;
        R_pose_sync = data.R_pose;
        timestamp = data.timestamp;
        last_collect_time = now;

        // GT state: zero-latency, Kalman-filtered velocity and angular velocity (always clean, noise-free)
        if (gt_state_enable && gt_state_pub) {
            double ts_sec = data.timestamp / 1e9;
            if (!gt_has_prev) {
                for (int ax = 0; ax < 3; ++ax)
                    kf_pos[ax].update(x_clean(ax), dt_desired, kf_q_pos, kf_r_pos);
                R_gt_prev   = R_clean;
                gt_prev_ts  = ts_sec;
                gt_has_prev = true;
            } else {
                double dt = ts_sec - gt_prev_ts;
                if (dt > 1e-6) {
                    Eigen::Vector3d v_filt;
                    for (int ax = 0; ax < 3; ++ax)
                        v_filt(ax) = kf_pos[ax].update(x_clean(ax), dt, kf_q_pos, kf_r_pos);

                    Eigen::Matrix3d Sk = R_clean.transpose() * ((R_clean - R_gt_prev) / dt);
                    Eigen::Vector3d omega_raw(
                        (Sk(2,1) - Sk(1,2)) / 2.0,
                        (Sk(0,2) - Sk(2,0)) / 2.0,
                        (Sk(1,0) - Sk(0,1)) / 2.0);
                    Eigen::Vector3d omega_filt;
                    for (int ax = 0; ax < 3; ++ax)
                        omega_filt(ax) = kf_omega[ax].update(omega_raw(ax), dt, kf_q_omega, kf_r_omega);

                    // EMA post-filter: second smoothing stage on top of Kalman
                    if (!smooth_has_prev) {
                        v_smooth_prev     = v_filt;
                        omega_smooth_prev = omega_filt;
                        smooth_has_prev   = true;
                    } else {
                        v_filt     = vel_smooth_alpha   * v_filt     + (1.0 - vel_smooth_alpha)   * v_smooth_prev;
                        omega_filt = omega_smooth_alpha * omega_filt + (1.0 - omega_smooth_alpha) * omega_smooth_prev;
                        v_smooth_prev     = v_filt;
                        omega_smooth_prev = omega_filt;
                    }

                    R_gt_prev  = R_clean;
                    gt_prev_ts = ts_sec;

                    const Eigen::Matrix3d& T = gt_transform_enable
                                               ? T_gt
                                               : Eigen::Matrix3d::Identity();
                    Eigen::Vector3d pos_out   = T * x_clean;
                    Eigen::Vector3d vel_out   = T * v_filt;
                    Eigen::Matrix3d R_out     = T * R_clean * T.transpose();
                    Eigen::Vector3d omega_out = T * omega_filt;

                    std::vector<std::vector<double>> R_mat(3, std::vector<double>(3));
                    for (int ri = 0; ri < 3; ++ri)
                        for (int ci = 0; ci < 3; ++ci)
                            R_mat[ri][ci] = R_out(ri, ci);

                    nlohmann::json jg;
                    jg["timestamp"] = ts_sec;
                    jg["rel_pos"]   = {pos_out(0),   pos_out(1),   pos_out(2)};
                    jg["rel_vel"]   = {vel_out(0),   vel_out(1),   vel_out(2)};
                    jg["rel_R"]     = R_mat;
                    jg["rel_omega"] = {omega_out(0), omega_out(1), omega_out(2)};
                    try {
                        gt_state_pub->put(jg.dump());
                    } catch (const std::exception& e) {
                        std::cerr << "VICON2POSE: [gt_state] publish error: " << e.what() << std::endl;
                    }
                }
            }
        }

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
                          << ", noise_angles: " << data.noise_angles.transpose()
                          << (position_only ? " [POSITION_ONLY mode]" : "") << std::endl;
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
    if (fast_publisher) {
        fast_publisher.reset();
    }
    if (gt_state_pub) {
        gt_state_pub.reset();
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