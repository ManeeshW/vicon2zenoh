#ifndef RELATIVE_POSE_HPP
#define RELATIVE_POSE_HPP

#include "vicon2pose.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <string>
#include <queue>
#include <deque>
#include <functional>
#include <random>
#include <nlohmann/json.hpp>

// ── RelativePose ─────────────────────────────────────────────────────────────
//
// Computes and publishes the relative pose between two vicon2pose objects.
// Reads x_clean_latest / R_clean_latest from base and rover directly
// (single-threaded: called after both trackers' loop() in main).
//
// Zenoh topics:
//   zenoh_key  — noisy, latency-delayed pose  (same JSON schema as vicon2pose)
//   fast_key   — clean, zero-latency fast pose
//   gt_state_key — Savitzky–Golay smoothed pos/vel/R/omega of the relative pose
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

    // Savitzky–Golay differentiator for the GT state: a sliding window of clean
    // relative-pose samples is fitted with a least-squares polynomial (in actual
    // time, robust to loop jitter) and evaluated at the window CENTER. Smoothed
    // value = poly(t_center), derivative = d(poly)/dt(t_center). This yields an
    // ultrasmooth, essentially noiseless velocity at the cost of a fixed latency
    // of half the window (published timestamp = center sample's real time, so
    // consumers can time-align). sg_window must be odd and >= sg_order + 2.
    int    sg_window     = 21;    // odd; span = sg_window/gt_freq, latency = half
    int    sg_order      = 3;     // polynomial order of the fit
    double sg_max_jump_m = 0.5;   // drop samples stepping farther than this from the
                                  // last accepted sample (Vicon occlusion glitches);
                                  // <= 0 disables the guard

    struct SgSample {
        double t;                 // real timestamp (s)
        Eigen::Vector3d x;        // relative position
        Eigen::Quaterniond q;     // relative attitude (hemisphere-continuous)
    };
    std::deque<SgSample> sg_buf;
    bool sg_has_prev_q = false;
    Eigen::Quaterniond sg_prev_q = Eigen::Quaterniond::Identity();

    // Least-squares polynomial fit (order sg_order, in tau = t - t_center) of one
    // channel over sg_buf, returning {value, derivative} at the window center.
    std::pair<double, double> sg_fit_center(
        const std::function<double(const SgSample&)>& channel) const;

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
