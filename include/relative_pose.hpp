#ifndef RELATIVE_POSE_HPP
#define RELATIVE_POSE_HPP

#include "vicon2pose.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <queue>
#include <random>
#include <nlohmann/json.hpp>

// ── Kalman filters (used only by RelativePose for velocity estimation) ──────

// 1-D Constant-Acceleration Kalman filter  state: [pos, vel, acc]  meas: pos
// gate_sigma > 0 clips the innovation to gate_sigma * sqrt(S) before applying the
// Kalman gain, so a single glitched/occluded measurement can't inject a step change
// (spike) into the state; 0 disables gating.
struct KalmanCA1D {
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 10.0;
    bool initialized  = false;
    double update(double z, double dt, double q, double r, double gate_sigma = 0.0);
};

// 1-D Constant-Velocity Kalman filter  state: [omega, alpha]  meas: omega
struct KalmanCV1D {
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity() * 10.0;
    bool initialized  = false;
    double update(double z, double dt, double q, double r, double gate_sigma = 0.0);
};

// ── RelativePose ─────────────────────────────────────────────────────────────
//
// Computes and publishes the relative pose between two vicon2pose objects.
// Reads x_clean_latest / R_clean_latest from base and rover directly
// (single-threaded: called after both trackers' loop() in main).
//
// Zenoh topics:
//   zenoh_key  — noisy, latency-delayed pose  (same JSON schema as vicon2pose)
//   fast_key   — clean, zero-latency fast pose
//   gt_state_key — Kalman-filtered pos/vel/R/omega of the relative pose
//
class RelativePose {
public:
    RelativePose(vicon2pose& base, vicon2pose& rover);
    ~RelativePose();

    bool on = false;

    // Optional display integration — set by main(), not owned here
    PoseSnap*  display_rel_snap = nullptr;  // updated at fast rate with clean relative pose
    FreqMeter* display_rel_freq = nullptr;  // ticked on each main-path zenoh publish
    GTSnap*    display_gt_snap  = nullptr;  // updated in publish_gt_state()
    FreqMeter* display_gt_freq  = nullptr;  // ticked in publish_gt_state()

    // Config fields accessed by main()
    std::string zenoh_key    = "fdcl/pose_sync";
    double      latency      = 0.0;
    double      frequency    = 5.0;
    std::string gt_state_key = "fdcl/rel_gt_state";
    bool        gt_state_enable = false;

    std::string meas_type_str() const {
        return meas_type_ == MeasType::BASE2ROVER ? "base2rover" : "rover2base";
    }

    void load_config(const std::string& config_file);
    void open();
    void loop();
    void close();

private:
    vicon2pose& base_;
    vicon2pose& rover_;

    enum class MeasType { ROVER2BASE, BASE2ROVER };
    MeasType meas_type_ = MeasType::ROVER2BASE;
    double dt_desired       = 0.2;
    double std_x            = 0.0;
    double std_R            = 0.0;
    bool noise_x_enabled    = false;
    bool noise_R_enabled    = false;
    bool position_only      = false;

    // Fast pose topic
    std::string fast_key = "fdcl/rel_pose_sync_fast";
    bool fast_enable     = false;
    double fast_dt       = 0.005; // 200 Hz default
    std::chrono::steady_clock::time_point fast_last_time;

    // GT state topic (gt_state_key and gt_state_enable are in public section)
    // Published at its own independent rate (gt_freq), decoupled from the noisy
    // zenoh_key topic's rate (frequency).
    double gt_freq              = 5.0;
    double gt_dt_desired        = 0.2;
    std::chrono::steady_clock::time_point gt_last_time;
    bool gt_transform_enable    = false;
    Eigen::Matrix3d T_gt        = Eigen::Matrix3d::Identity();
    double kf_q_pos             = 0.01;
    double kf_r_pos             = 1e-6;
    double kf_q_omega           = 0.5;
    double kf_r_omega           = 0.04;
    // Spike rejection: clip innovation to this many sigma before it can move the
    // filter state. 0 disables gating.
    double kf_gate_sigma_pos    = 5.0;
    double kf_gate_sigma_omega  = 5.0;
    // EMA post-filter smoothing, expressed as a time constant (seconds) rather than
    // a fixed per-sample alpha, so smoothing strength stays consistent regardless of
    // gt_freq. Effective alpha = 1 - exp(-dt / tau); larger tau = smoother, more lag.
    double vel_smooth_tau_sec   = 0.5;
    double omega_smooth_tau_sec = 0.3;

    KalmanCA1D kf_pos[3];
    KalmanCV1D kf_omega[3];
    Eigen::Vector3d v_smooth_prev     = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_smooth_prev = Eigen::Vector3d::Zero();
    bool smooth_has_prev  = false;
    Eigen::Matrix3d R_gt_prev = Eigen::Matrix3d::Identity();
    double gt_prev_ts    = 0.0;
    bool gt_has_prev     = false;

    // Zenoh
    std::optional<zenoh::Session>   session;
    std::optional<zenoh::Publisher> publisher;
    std::optional<zenoh::Publisher> fast_publisher;
    std::optional<zenoh::Publisher> gt_state_pub;

    struct PoseData {
        Eigen::Vector3d x_pose;
        Eigen::Matrix3d R_pose;
        double timestamp;
        std::chrono::steady_clock::time_point collect_time;
        Eigen::Vector3d noise_x;
        Eigen::Vector3d noise_angles;
    };
    std::queue<PoseData> pose_buffer;
    std::chrono::steady_clock::time_point last_collect_time;

    std::mt19937 rng;
    std::normal_distribution<double> dist;

    // Compute relative pose (x_rel, R_rel) from clean base and rover poses
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>
    compute_relative(const Eigen::Vector3d& x_b, const Eigen::Matrix3d& R_b,
                     const Eigen::Vector3d& x_r, const Eigen::Matrix3d& R_r) const;

    // Run GT state estimation and publish; uses clean relative pose (no noise)
    void publish_gt_state(const Eigen::Vector3d& x_rel, const Eigen::Matrix3d& R_rel, double ts_sec);
};

#endif
