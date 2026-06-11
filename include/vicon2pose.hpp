#ifndef VICON2POSE_HPP
#define VICON2POSE_HPP

#include "vicon.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <nlohmann/json.hpp>
#include <queue>
#include <random>
#include <vector>
#include <memory>
#include <optional>
#include <map>

struct TrackerConfig {
    double latency         = 0.0;
    double dt_desired      = 0.01;
    double std_x           = 0.0;
    double std_R           = 0.0;
    bool   noise_x_enabled = false;
    bool   noise_R_enabled = false;
};

// ---------------------------------------------------------------------------
// Kalman filters for velocity / angular-velocity smoothing
// ---------------------------------------------------------------------------

/**
 * 1-D Constant-Acceleration Kalman filter.
 * State: [pos, vel, acc].  Measurement: position.
 * Returns filtered velocity on each call to update().
 */
struct KalmanCA1D {
    Eigen::Vector3d x = Eigen::Vector3d::Zero();   // [pos, vel, acc]
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 10.0;
    bool initialized  = false;

    double update(double z, double dt, double q, double r);
};

/**
 * 1-D Constant-Velocity Kalman filter.
 * State: [omega, alpha].  Measurement: raw finite-difference omega.
 * Returns smoothed omega on each call to update().
 */
struct KalmanCV1D {
    Eigen::Vector2d x = Eigen::Vector2d::Zero();   // [omega, angular_acc]
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity() * 10.0;
    bool initialized  = false;

    double update(double z, double dt, double q, double r);
};

// ---------------------------------------------------------------------------

/** Tracks one Vicon object and publishes its pose to a dedicated Zenoh key. */
class ObjectTracker {
public:
    ObjectTracker(const std::string& name, const std::string& key,
                  zenoh::Session& session, const TrackerConfig& cfg);
    ~ObjectTracker() = default;

    void loop();
    void close();

    const std::string name;
    const std::string key;
    Eigen::Vector3d   x_pose    = Eigen::Vector3d::Zero();
    Eigen::Matrix3d   R_pose    = Eigen::Matrix3d::Identity();
    double            timestamp = 0.0;
    bool              has_data  = false;
    bool              updated   = false; ///< true for one loop() after new Vicon data arrives

private:
    struct PoseData {
        Eigen::Vector3d x_pose;
        Eigen::Matrix3d R_pose;
        double          timestamp;
        std::chrono::steady_clock::time_point collect_time;
        Eigen::Vector3d noise_x;
        Eigen::Vector3d noise_angles;
    };

    double latency;
    double dt_desired;
    double std_x;
    double std_R;
    bool   noise_x_enabled;
    bool   noise_R_enabled;

    Eigen::Matrix3d vicon_to_body; // R_off * R_sv

    vicon                            vicon_instance;
    std::optional<zenoh::Publisher>  publisher;
    std::queue<PoseData>             pose_buffer;
    std::chrono::steady_clock::time_point last_collect_time;
    std::mt19937                     rng;
    std::normal_distribution<double> dist;
};

/**
 * Manages multiple ObjectTrackers and publishes relative pose to fdcl/pose_sync.
 *
 * Each object publishes to fdcl/object1, fdcl/object2, etc.
 * The relative pose is: pose of object[pose_sync_to] expressed in the
 * frame of object[pose_sync_from]:
 *   x_rel = R_from.T * (x_to - x_from)
 *   R_rel = R_from.T * R_to
 *
 * fdcl/rel_gt_state also publishes with Kalman-smoothed velocity and an
 * optional output frame transform T_rel (applied as: p' = T*p, R' = T*R*T^T).
 */
class vicon2pose {
public:
    vicon2pose();
    ~vicon2pose();

    bool on = false;

    void load_config(const std::string& config_file);
    void open();
    void loop();
    void close();

private:
    double latency         = 0.0;
    double frequency       = 200.0;
    double dt_desired      = 0.005;
    double std_x           = 0.0;
    double std_R           = 0.0;
    bool   noise_x_enabled = false;
    bool   noise_R_enabled = false;

    struct PerObjectOverride {
        std::optional<double> frequency;
        std::optional<double> latency;
        std::optional<double> std_x;
        std::optional<double> std_R;
        std::optional<bool>   noise_x_enabled;
        std::optional<bool>   noise_R_enabled;
    };
    std::map<int, PerObjectOverride> object_overrides;

    std::vector<std::string> object_names;
    std::vector<std::string> object_keys;
    int         pose_sync_from = 0;
    int         pose_sync_to   = 1;
    std::string pose_sync_key  = "fdcl/pose_sync";

    // rel_gt_state settings
    std::string rel_gt_state_key    = "fdcl/rel_gt_state";
    bool        rel_gt_state_enable = true;

    // Output-frame transform for rel_gt_state (T @ pos, T @ R @ T^T, T @ vel, T @ omega)
    bool            rel_pose_transform_enable = true;
    Eigen::Matrix3d T_rel = (Eigen::Matrix3d() << 0,1,0, 1,0,0, 0,0,1).finished();

    // Kalman filter parameters
    double kf_q_pos   = 1.0;     // CA process noise spectral density (m/s^3)
    double kf_r_pos   = 1e-6;    // position measurement noise variance (m^2)
    double kf_q_omega = 5.0;     // CV omega process noise spectral density (rad/s^2)
    double kf_r_omega = 0.04;    // omega measurement noise variance (rad/s)^2

    // Per-axis Kalman filters
    KalmanCA1D kf_pos[3];
    KalmanCV1D kf_omega[3];

    // Previous relative pose for angular velocity computation
    Eigen::Matrix3d R_rel_prev   = Eigen::Matrix3d::Identity();
    double          rel_prev_ts  = 0.0;
    bool            rel_has_prev = false;

    std::vector<std::unique_ptr<ObjectTracker>> trackers;
    std::optional<zenoh::Session>   session;
    std::optional<zenoh::Publisher> sync_publisher;
    std::optional<zenoh::Publisher> rel_gt_state_pub;

    struct RelPoseData {
        Eigen::Vector3d x_rel;
        Eigen::Matrix3d R_rel;
        double          timestamp;
        std::chrono::steady_clock::time_point collect_time;
    };
    std::queue<RelPoseData> rel_buffer;

    void queue_and_drain_relative_pose();
};

#endif
