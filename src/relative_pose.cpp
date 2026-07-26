#include "relative_pose.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>

// ── Savitzky–Golay fit helper ────────────────────────────────────────────────
// Least-squares fit of a polynomial of order sg_order to one channel of the
// window samples, in tau = t - t_center. Returns {value, derivative} evaluated
// at the window center (tau = 0), i.e. coefficients c0 and c1 of the fit.
// Fitting against actual timestamps makes the derivative robust to the small
// period jitter of the publish loop.
std::pair<double, double> RelativePose::sg_fit_center(
    const std::function<double(const SgSample&)>& channel) const {
    const int n = static_cast<int>(sg_buf.size());
    const int c = n / 2;                 // center index (n is odd)
    const double tc = sg_buf[c].t;
    Eigen::MatrixXd V(n, sg_order + 1);
    Eigen::VectorXd y(n);
    for (int i = 0; i < n; ++i) {
        double tau = sg_buf[i].t - tc;
        double p = 1.0;
        for (int k = 0; k <= sg_order; ++k) { V(i, k) = p; p *= tau; }
        y(i) = channel(sg_buf[i]);
    }
    Eigen::VectorXd coef = V.colPivHouseholderQr().solve(y);
    return {coef(0), coef(1)};
}

// ── RelativePose ─────────────────────────────────────────────────────────────

RelativePose::RelativePose(vicon2pose& base, vicon2pose& rover)
    : base_(base), rover_(rover),
      fast_last_time(std::chrono::steady_clock::now()),
      last_collect_time(std::chrono::steady_clock::now()),
      gt_last_time(std::chrono::steady_clock::now()),
      rng(std::random_device{}()), dist(0.0, 1.0) {}

RelativePose::~RelativePose() {
    close();
}

static std::string rel_trim_value(const std::string& s) {
    auto cpos = s.find('#');
    std::string r = (cpos != std::string::npos) ? s.substr(0, cpos) : s;
    auto e = r.find_last_not_of(" \t\r\n");
    return (e == std::string::npos) ? "" : r.substr(0, e + 1);
}

void RelativePose::load_config(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "[Relative] Could not open config file: " << config_file << std::endl;
        return;
    }

    bool in_section = false;
    std::string line;
    while (std::getline(file, line)) {
        auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);
        if (line[0] == '#') continue;

        if (line[0] == '[') {
            auto end = line.find(']');
            if (end != std::string::npos) {
                std::string sec = line.substr(1, end - 1);
                auto s0 = sec.find_first_not_of(" \t");
                auto s1 = sec.find_last_not_of(" \t");
                sec = (s0 == std::string::npos) ? "" : sec.substr(s0, s1 - s0 + 1);
                in_section = (sec == "Relative");
            }
            continue;
        }
        if (!in_section) continue;

        auto sep = line.find_first_of(":=");
        if (sep == std::string::npos) continue;
        std::string key = line.substr(0, sep);
        std::string val = line.substr(sep + 1);

        auto k0 = key.find_first_not_of(" \t");
        auto k1 = key.find_last_not_of(" \t");
        if (k0 == std::string::npos) continue;
        key = key.substr(k0, k1 - k0 + 1);

        auto v0 = val.find_first_not_of(" \t");
        val = (v0 == std::string::npos) ? "" : val.substr(v0);
        val = rel_trim_value(val);

        if (key == "on") {
            on = (val == "true" || val == "1");
        } else if (key == "Measurement_type") {
            // left  = rover2base (rover pose expressed in base/body frame) — default
            // right = base2rover (base pose expressed in rover frame)
            meas_type_ = (val == "base2rover" || val == "right") ? MeasType::BASE2ROVER : MeasType::ROVER2BASE;
        } else if (key == "zenoh_key") {
            zenoh_key = val;
        } else if (key == "latency") {
            try { latency = std::stod(val); } catch (...) { latency = 0.0; }
        } else if (key == "frequency") {
            try { frequency = std::stod(val); dt_desired = 1.0 / frequency; }
            catch (...) { frequency = 5.0; dt_desired = 0.2; }
        } else if (key == "std_x") {
            try { std_x = std::stod(val); } catch (...) {}
        } else if (key == "std_R") {
            try { std_R = std::stod(val); } catch (...) {}
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
            try { double ff = std::stod(val); if (ff > 0.0) fast_dt = 1.0 / ff; } catch (...) {}
        } else if (key == "gt_state_key") {
            gt_state_key = val;
        } else if (key == "gt_state_enable") {
            gt_state_enable = (val == "true" || val == "1");
        } else if (key == "gt_freq") {
            try {
                double gf = std::stod(val);
                if (gf > 0.0) { gt_freq = gf; gt_dt_desired = 1.0 / gf; }
            } catch (...) {}
        } else if (key == "gt_transform_enable") {
            gt_transform_enable = (val == "true" || val == "1");
        } else if (key == "gt_transform_matrix") {
            std::stringstream ss(val);
            std::string tok;
            std::vector<double> vals;
            while (std::getline(ss, tok, ',')) {
                auto t0 = tok.find_first_not_of(" \t");
                auto t1 = tok.find_last_not_of(" \t");
                if (t0 == std::string::npos) continue;
                try { vals.push_back(std::stod(tok.substr(t0, t1 - t0 + 1))); } catch (...) {}
            }
            if (vals.size() == 9)
                for (int ri = 0; ri < 3; ++ri)
                    for (int ci = 0; ci < 3; ++ci)
                        T_gt(ri, ci) = vals[ri*3 + ci];
            else
                std::cerr << "[Relative] gt_transform_matrix needs 9 values, got " << vals.size() << std::endl;
        } else if (key == "gt_sg_window") {
            try {
                int w = std::stoi(val);
                if (w % 2 == 0) ++w;                    // window must be odd
                if (w >= sg_order + 2) sg_window = w;
                else std::cerr << "[Relative] gt_sg_window too small, keeping " << sg_window << std::endl;
            } catch (...) {}
        } else if (key == "gt_sg_order") {
            try {
                int p = std::stoi(val);
                if (p >= 1 && p + 2 <= sg_window) sg_order = p;
                else std::cerr << "[Relative] gt_sg_order invalid, keeping " << sg_order << std::endl;
            } catch (...) {}
        } else if (key == "gt_sg_max_jump_m") {
            try { sg_max_jump_m = std::stod(val); } catch (...) {}
        }
    }
    file.close();

    std::string mtype = (meas_type_ == MeasType::ROVER2BASE) ? "rover2base" : "base2rover";
    std::cout << "[Relative] Config: type=" << mtype
              << " zenoh_key=" << zenoh_key
              << " on=" << on << " frequency=" << frequency << "Hz"
              << " latency=" << latency << "s"
              << " fast=" << (fast_enable ? fast_key : std::string("off"))
              << " gt=" << (gt_state_enable ? gt_state_key : std::string("off"))
              << (gt_state_enable ? (" @" + std::to_string(gt_freq) + "Hz") : std::string())
              << (gt_state_enable ? (" sg=" + std::to_string(sg_window) + "/" + std::to_string(sg_order)) : std::string()) << std::endl;
}

void RelativePose::open() {
    zenoh::ZResult* err = nullptr;
    try {
        auto config = zenoh::Config::create_default();
        session = zenoh::Session(std::move(config), zenoh::Session::SessionOptions::create_default(), err);
        publisher = session->declare_publisher(zenoh_key, zenoh::Session::PublisherOptions::create_default(), err);
        std::cout << "RELATIVE: publisher on " << zenoh_key << std::endl;
        if (fast_enable) {
            fast_publisher = session->declare_publisher(fast_key, zenoh::Session::PublisherOptions::create_default(), err);
            std::cout << "RELATIVE: fast publisher on " << fast_key
                      << " (" << static_cast<int>(1.0 / fast_dt) << " Hz)" << std::endl;
        }
        if (gt_state_enable) {
            gt_state_pub = session->declare_publisher(gt_state_key, zenoh::Session::PublisherOptions::create_default(), err);
            std::cout << "RELATIVE: GT state publisher on " << gt_state_key << std::endl;
        }
        on = true;
    } catch (const std::exception& e) {
        std::cerr << "RELATIVE: Failed to open Zenoh - " << e.what() << std::endl;
        on = false;
    }
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d>
RelativePose::compute_relative(const Eigen::Vector3d& x_b, const Eigen::Matrix3d& R_b,
                                const Eigen::Vector3d& x_r, const Eigen::Matrix3d& R_r) const {
    if (meas_type_ == MeasType::ROVER2BASE) {
        // Position and attitude of rover expressed in base frame
        return { R_b.transpose() * (x_r - x_b), R_b.transpose() * R_r };
    } else {
        // Position and attitude of base expressed in rover frame
        return { R_r.transpose() * (x_b - x_r), R_r.transpose() * R_b };
    }
}

void RelativePose::publish_gt_state(const Eigen::Vector3d& x_rel, const Eigen::Matrix3d& R_rel, double ts_sec) {
    if (!gt_state_pub) return;

    // Glitch guard: drop samples that step implausibly far from the last accepted
    // one (Vicon occlusion jumps) so they can't smear across the whole SG window.
    if (sg_max_jump_m > 0.0 && !sg_buf.empty() &&
        (x_rel - sg_buf.back().x).norm() > sg_max_jump_m)
        return;

    // Push the clean sample; keep the quaternion hemisphere-continuous so the
    // component-wise polynomial fit sees a smooth signal.
    Eigen::Quaterniond q_rel(R_rel);
    if (sg_has_prev_q && q_rel.coeffs().dot(sg_prev_q.coeffs()) < 0.0)
        q_rel.coeffs() *= -1.0;
    sg_prev_q     = q_rel;
    sg_has_prev_q = true;
    sg_buf.push_back({ts_sec, x_rel, q_rel});
    if (static_cast<int>(sg_buf.size()) > sg_window) sg_buf.pop_front();
    if (static_cast<int>(sg_buf.size()) < sg_window) return;   // window not full yet

    // Fit each channel over the window; value + derivative at the window center.
    Eigen::Vector3d pos_sm, vel_sm;
    for (int ax = 0; ax < 3; ++ax) {
        auto [v, d] = sg_fit_center([ax](const SgSample& s) { return s.x(ax); });
        pos_sm(ax) = v; vel_sm(ax) = d;
    }
    // Quaternion coeffs() layout is (x, y, z, w); Quaterniond ctor takes (w, x, y, z).
    Eigen::Vector4d qc_sm, qcd_sm;
    for (int k = 0; k < 4; ++k) {
        auto [v, d] = sg_fit_center([k](const SgSample& s) { return s.q.coeffs()(k); });
        qc_sm(k) = v; qcd_sm(k) = d;
    }
    Eigen::Quaterniond q_fit(qc_sm(3), qc_sm(0), qc_sm(1), qc_sm(2));
    q_fit.normalize();
    Eigen::Quaterniond q_dot(qcd_sm(3), qcd_sm(0), qcd_sm(1), qcd_sm(2));

    // Body-frame angular velocity: R_dot = R [w]x  <=>  w = 2 * vec(q* ⊗ q_dot).
    // Same convention as the previous R^T*dR finite-difference estimate.
    Eigen::Vector3d omega_sm = 2.0 * (q_fit.conjugate() * q_dot).vec();

    // The published sample belongs to the window center: its timestamp is the
    // center sample's real time (latency = half the window, explicit).
    double ts_center = sg_buf[sg_buf.size() / 2].t;

    const Eigen::Matrix3d& T = gt_transform_enable ? T_gt : Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos_out   = T * pos_sm;
    Eigen::Vector3d vel_out   = T * vel_sm;
    Eigen::Matrix3d R_out     = T * q_fit.toRotationMatrix() * T.transpose();
    Eigen::Vector3d omega_out = T * omega_sm;

    std::vector<std::vector<double>> R_mat(3, std::vector<double>(3));
    for (int ri = 0; ri < 3; ++ri)
        for (int ci = 0; ci < 3; ++ci)
            R_mat[ri][ci] = R_out(ri, ci);

    nlohmann::json jg;
    jg["timestamp"] = ts_center;
    jg["rel_pos"]   = {pos_out(0),   pos_out(1),   pos_out(2)};
    jg["rel_vel"]   = {vel_out(0),   vel_out(1),   vel_out(2)};
    jg["rel_R"]     = R_mat;
    jg["rel_omega"] = {omega_out(0), omega_out(1), omega_out(2)};
    try {
        gt_state_pub->put(jg.dump());
        if (display_gt_snap) display_gt_snap->update(pos_out, vel_out, R_out, omega_out);
        if (display_gt_freq) display_gt_freq->tick();
        if (!display_gt_snap) {
            std::cout << "[GT  -> " << gt_state_key << "]"
                      << "  pos: [" << pos_out(0) << ", " << pos_out(1) << ", " << pos_out(2) << "]"
                      << "  vel: [" << vel_out(0) << ", " << vel_out(1) << ", " << vel_out(2) << "]"
                      << "  omega: [" << omega_out(0) << ", " << omega_out(1) << ", " << omega_out(2) << "]"
                      << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "RELATIVE: [gt_state] publish error: " << e.what() << std::endl;
    }
}

void RelativePose::loop() {
    if (!on) return;
    if (!base_.has_data || !rover_.has_data) return;

    auto now = std::chrono::steady_clock::now();

    // Fast path: publish clean relative pose at fast rate
    if (fast_enable && fast_publisher &&
        std::chrono::duration<double>(now - fast_last_time).count() >= fast_dt) {
        auto [xf, Rf] = compute_relative(
            base_.x_clean_latest, base_.R_clean_latest,
            rover_.x_clean_latest, rover_.R_clean_latest);

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
        try { fast_publisher->put(jf.dump()); }
        catch (const std::exception& e) {
            std::cerr << "RELATIVE: [fast] publish error: " << e.what() << std::endl;
        }
        if (display_rel_snap) display_rel_snap->update(xf, Rf);
        fast_last_time = now;
    }

    // GT state: published at its own independent rate (gt_freq), decoupled from
    // the noisy zenoh_key topic's rate. Always uses clean, noise-free relative pose.
    if (gt_state_enable && gt_state_pub &&
        std::chrono::duration<double>(now - gt_last_time).count() >= gt_dt_desired) {
        auto [x_gt_clean, R_gt_clean] = compute_relative(
            base_.x_clean_latest, base_.R_clean_latest,
            rover_.x_clean_latest, rover_.R_clean_latest);

        double ts_gt = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        publish_gt_state(x_gt_clean, R_gt_clean, ts_gt / 1e9);
        gt_last_time = now;
    }

    // Main path: collect at configured frequency
    if (std::chrono::duration<double>(now - last_collect_time).count() >= dt_desired) {
        auto [x_rel_clean, R_rel_clean] = compute_relative(
            base_.x_clean_latest, base_.R_clean_latest,
            rover_.x_clean_latest, rover_.R_clean_latest);

        double ts = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        PoseData data;
        data.x_pose       = x_rel_clean;
        data.R_pose       = position_only ? -Eigen::Matrix3d::Identity() : R_rel_clean;
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

        pose_buffer.push(data);
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
            if (display_rel_freq) display_rel_freq->tick();
            // Also update snap for the no-fast-enable case
            if (display_rel_snap && !fast_enable)
                display_rel_snap->update(front.x_pose, front.R_pose);
            if (!display_rel_snap) {
                std::cout << "[Rel -> " << zenoh_key << "]"
                          << "  rel_pos: [" << front.x_pose(0) << ", "
                                            << front.x_pose(1) << ", "
                                            << front.x_pose(2) << "]"
                          << "  R(row0): [" << front.R_pose(0,0) << ", "
                                            << front.R_pose(0,1) << ", "
                                            << front.R_pose(0,2) << "]"
                          << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "RELATIVE: Publish error - " << e.what() << std::endl;
        }
        pose_buffer.pop();
    }
}

void RelativePose::close() {
    on = false;
    publisher.reset();
    fast_publisher.reset();
    gt_state_pub.reset();
    if (session) {
        session->close(zenoh::Session::SessionCloseOptions::create_default(), nullptr);
        session.reset();
    }
    while (!pose_buffer.empty()) pose_buffer.pop();
    std::cout << "RELATIVE: Closed" << std::endl;
}
