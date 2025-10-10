#ifndef VICON2GT_HPP
#define VICON2GT_HPP

#include "vicon.hpp"
#include <zenoh.hxx>
#include <Eigen/Dense>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <string>
#include <nlohmann/json.hpp>
#include <queue>

class vicon2gt {
public:
    vicon2gt();
    ~vicon2gt();

    bool on = false; /**< Flag to enable/disable vicon2gt */
    std::string zenoh_key = "gt"; /**< Zenoh publisher key */
    double latency = 0.0; /**< Latency in seconds for delayed publishing */
    double frequency = 200.0; /**< Frequency in Hz for data collection */
    double dt_desired = 0.005; /**< Desired loop time (1/frequency) for data collection */
    double alpha = 0.1; /**< Alpha for lowpass filter on velocity */
    Eigen::Matrix3d R_sv = Eigen::Matrix3d::Identity(); /**< Transformation matrix from Vicon to ship frame */
    Eigen::Matrix3d R_off = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity(); /**< Additional transformation matrix */
    Eigen::Vector3d x_pose_sync = Eigen::Vector3d::Zero(); /**< Position in ship frame */
    Eigen::Matrix3d R_pose_sync = Eigen::Matrix3d::Identity(); /**< Rotation in ship frame */
    Eigen::Vector3d v_pose_sync = Eigen::Vector3d::Zero(); /**< Velocity in ship frame */
    double timestamp = 0.0; /**< Timestamp of last gt data */

    std::mutex gt_mutex; /**< Mutex for thread-safe access to gt data */
    std::condition_variable data_cv; /**< Condition variable to notify new data */
    std::chrono::steady_clock::time_point last_collect_time; /**< Time of last data collection */

    void load_config(const std::string& config_file);
    void open();
    void loop();
    void close();

private:
    struct GtData {
        Eigen::Vector3d x_pose;
        Eigen::Matrix3d R_pose;
        Eigen::Vector3d velocity;
        double timestamp;  // In seconds
        std::chrono::steady_clock::time_point collect_time;
    };

    vicon vicon_instance; /**< Vicon instance for data acquisition */
    std::optional<zenoh::Session> session; /**< Zenoh session */
    std::optional<zenoh::Publisher> publisher; /**< Zenoh publisher */
    std::queue<GtData> gt_buffer; /**< Buffer to store gt data with timestamps */
    Eigen::Vector3d prev_x_pose = Eigen::Vector3d::Zero();
    double prev_timestamp = 0.0;
    Eigen::Vector3d prev_velocity = Eigen::Vector3d::Zero();
    bool first_measurement = true;
};

#endif