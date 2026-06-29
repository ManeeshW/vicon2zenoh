#ifndef VICON2POSE_HPP
#define VICON2POSE_HPP

#include "vicon.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <string>
#include <nlohmann/json.hpp>
#include <queue>
#include <random>

// 1-D Constant-Acceleration Kalman filter (state: [pos, vel, acc], measurement: pos)
struct KalmanCA1D {
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 10.0;
    bool initialized  = false;
    double update(double z, double dt, double q, double r);
};

// 1-D Constant-Velocity Kalman filter (state: [omega, alpha], measurement: raw omega)
struct KalmanCV1D {
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity() * 10.0;
    bool initialized  = false;
    double update(double z, double dt, double q, double r);
};

class vicon2pose {
public:
    vicon2pose();
    ~vicon2pose();

    bool on = false; /**< Flag to enable/disable vicon2pose */
    std::string zenoh_key = "fdcl/pose_sync"; /**< Zenoh publisher key */
    double latency = 0.5; /**< Latency in seconds for delayed publishing */
    double frequency = 5.0; /**< Frequency in Hz for data collection */
    double dt_desired = 0.2; /**< Desired loop time (1/frequency) for data collection */
    double std_x = 0.0; /**< Standard deviation for position noise */
    double std_R = 0.0; /**< Standard deviation for rotation noise */
    bool noise_x_enabled = false; /**< Flag to enable/disable position noise */
    bool noise_R_enabled = false; /**< Flag to enable/disable rotation noise */
    bool position_only = false; /**< NEW: When true, send real position + fake attitude (keeps JSON compatible) */
    Eigen::Matrix3d R_sv = Eigen::Matrix3d::Identity(); /**< Transformation matrix from Vicon to ship frame */
    Eigen::Matrix3d  R_off = Eigen::Matrix3d::Identity();
    Eigen::Vector3d x_pose_sync = Eigen::Vector3d::Zero(); /**< Position in ship frame */
    Eigen::Matrix3d R_pose_sync = Eigen::Matrix3d::Identity(); /**< Rotation in ship frame */
    double timestamp = 0.0; /**< Timestamp of last pose data */

    std::mutex pose_mutex; /**< Mutex for thread-safe access to pose data */
    std::condition_variable data_cv; /**< Condition variable to notify new data */
    std::chrono::steady_clock::time_point last_collect_time; /**< Time of last data collection */

    void load_config(const std::string& config_file);
    void open();
    void loop();
    void close();

private:
    struct PoseData {
        Eigen::Vector3d x_pose;
        Eigen::Matrix3d R_pose;
        double timestamp;
        std::chrono::steady_clock::time_point collect_time;
        Eigen::Vector3d noise_x; /**< Noise applied to position */
        Eigen::Vector3d noise_angles; /**< Noise angles applied to rotation */
    };

    vicon vicon_instance; /**< Vicon instance for data acquisition */
    std::optional<zenoh::Session> session; /**< Zenoh session */
    std::optional<zenoh::Publisher> publisher; /**< Zenoh publisher */
    std::queue<PoseData> pose_buffer; /**< Buffer to store pose data with timestamps */
    std::mt19937 rng; /**< Random number generator for noise */
    std::normal_distribution<double> dist; /**< Normal distribution for noise */

    // Fast publisher (no noise, no latency, independent 200 Hz rate)
    std::string     fast_key    = "fdcl/pose_sync_fast";
    bool            fast_enable = false;
    double          fast_dt     = 0.005; // 200 Hz
    std::chrono::steady_clock::time_point fast_last_collect_time;
    std::optional<zenoh::Publisher> fast_publisher;

    // GT state (Kalman-filtered pos/vel/R/omega, zero-latency)
    std::string     gt_state_key        = "fdcl/gt_state";
    bool            gt_state_enable     = false;
    bool            gt_transform_enable = false;
    Eigen::Matrix3d T_gt                = Eigen::Matrix3d::Identity();
    double          kf_q_pos            = 1.0;
    double          kf_r_pos            = 1e-6;
    double          kf_q_omega          = 5.0;
    double          kf_r_omega          = 0.04;
    double          vel_smooth_alpha    = 1.0;   // EMA alpha for velocity (1=off, <1=smoother)
    double          omega_smooth_alpha  = 1.0;   // EMA alpha for omega   (1=off, <1=smoother)
    KalmanCA1D      kf_pos[3];
    KalmanCV1D      kf_omega[3];
    Eigen::Vector3d v_smooth_prev       = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_smooth_prev   = Eigen::Vector3d::Zero();
    bool            smooth_has_prev     = false;
    Eigen::Matrix3d R_gt_prev           = Eigen::Matrix3d::Identity();
    double          gt_prev_ts          = 0.0;
    bool            gt_has_prev         = false;
    std::optional<zenoh::Publisher> gt_state_pub;
};

#endif