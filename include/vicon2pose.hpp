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
    Eigen::Matrix3d R_sv = Eigen::Matrix3d::Identity(); /**< Transformation matrix from Vicon to ship frame */
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
};

#endif