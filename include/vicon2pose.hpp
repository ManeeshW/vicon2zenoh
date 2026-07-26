#ifndef VICON2POSE_HPP
#define VICON2POSE_HPP

#include "vicon.hpp"
#include "display.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <string>
#include <nlohmann/json.hpp>
#include <queue>
#include <random>

class vicon2pose {
public:
    vicon2pose();
    ~vicon2pose();

    bool on = false;
    std::string name;          // section name ("Base" or "Rover"), set by load_config
    std::string object_name;
    int port = 3883;
    std::string zenoh_key = "fdcl/pose_sync";
    double latency = 0.0;
    double frequency = 5.0;
    double dt_desired = 0.2;
    double std_x = 0.0;
    double std_R = 0.0;
    bool noise_x_enabled = false;
    bool noise_R_enabled = false;
    bool position_only = false;

    Eigen::Matrix3d R_sv;
    Eigen::Matrix3d R_off;

    // NED-frame conversion: R_nv (vicon/ENU -> NED), R_im (marker -> IMU mounting).
    // Both are self-inverse/symmetric, so the same matrix works in either direction.
    // Used for all main-path output and (unless fast_legacy_frame) fast-path output.
    Eigen::Matrix3d R_nv;
    Eigen::Matrix3d R_im;

    // When true, the fast_key publish keeps the legacy (pre-NED) R_sv/R_off transform
    // instead of the NED transform. x_clean_latest is always NED regardless of this flag.
    bool fast_legacy_frame = false;
    Eigen::Vector3d x_pose_sync = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_pose_sync = Eigen::Matrix3d::Identity();
    double timestamp = 0.0;

    // Clean (pre-noise) pose — updated at both fast and main rates; used by RelativePose
    Eigen::Vector3d x_clean_latest = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_clean_latest = Eigen::Matrix3d::Identity();
    double timestamp_clean_latest = 0.0;
    bool has_data = false;

    // Optional display integration — set by main(), not owned here
    PoseSnap*  display_snap = nullptr;  // updated at fast rate with clean pose
    FreqMeter* display_freq = nullptr;  // ticked on each main-path zenoh publish

    std::mutex pose_mutex;
    std::condition_variable data_cv;
    std::chrono::steady_clock::time_point last_collect_time;

    // load_config parses only the named section from an INI-style config file
    void load_config(const std::string& config_file, const std::string& section);
    void open();
    void loop();
    void close();

private:
    struct PoseData {
        Eigen::Vector3d x_pose;
        Eigen::Matrix3d R_pose;
        double timestamp;
        std::chrono::steady_clock::time_point collect_time;
        Eigen::Vector3d noise_x;
        Eigen::Vector3d noise_angles;
    };

    vicon vicon_instance;
    std::optional<zenoh::Session> session;
    std::optional<zenoh::Publisher> publisher;
    std::queue<PoseData> pose_buffer;
    std::mt19937 rng;
    std::normal_distribution<double> dist;

    // Fast publisher: no noise, no latency, independent rate
    std::string fast_key = "fdcl/pose_sync_fast";
    bool fast_enable = false;
    double fast_dt = 0.005; // 200 Hz default
    std::chrono::steady_clock::time_point fast_last_collect_time;
    std::optional<zenoh::Publisher> fast_publisher;
};

#endif
