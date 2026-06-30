#pragma once
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <cstdio>
#include <algorithm>

// ── Rolling frequency meter (thread-safe) ───────────────────────────────────
struct FreqMeter {
    static constexpr int N = 40;
    std::array<std::chrono::steady_clock::time_point, N> buf{};
    int idx   = 0;
    int count = 0;
    std::mutex mtx;

    void tick() {
        std::lock_guard<std::mutex> lg(mtx);
        buf[idx % N] = std::chrono::steady_clock::now();
        ++idx;
        if (count < N) ++count;
    }

    double hz() {
        std::lock_guard<std::mutex> lg(mtx);
        if (count < 2) return 0.0;
        int n       = std::min(count, N);
        auto newest = buf[(idx - 1 + N * 1000) % N];
        auto oldest = buf[(idx - n + N * 1000) % N];
        double dt   = std::chrono::duration<double>(newest - oldest).count();
        return dt > 1e-9 ? (n - 1.0) / dt : 0.0;
    }
};

// ── Thread-safe pose snapshot ────────────────────────────────────────────────
struct PoseSnap {
    std::mutex       mtx;
    Eigen::Vector3d  pos      = Eigen::Vector3d::Zero();
    Eigen::Matrix3d  rot      = Eigen::Matrix3d::Identity();
    bool             has_data = false;

    void update(const Eigen::Vector3d& p, const Eigen::Matrix3d& R) {
        std::lock_guard<std::mutex> lg(mtx);
        pos = p; rot = R; has_data = true;
    }

    struct Snap { Eigen::Vector3d pos; Eigen::Matrix3d rot; bool has_data; };
    Snap snap() {
        std::lock_guard<std::mutex> lg(mtx);
        return {pos, rot, has_data};
    }
};

// ── Thread-safe GT state snapshot ────────────────────────────────────────────
struct GTSnap {
    std::mutex       mtx;
    Eigen::Vector3d  pos      = Eigen::Vector3d::Zero();
    Eigen::Vector3d  vel      = Eigen::Vector3d::Zero();
    Eigen::Vector3d  omega    = Eigen::Vector3d::Zero();
    Eigen::Matrix3d  rot      = Eigen::Matrix3d::Identity();
    bool             has_data = false;

    void update(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& w) {
        std::lock_guard<std::mutex> lg(mtx);
        pos = p; vel = v; rot = R; omega = w; has_data = true;
    }

    struct Snap {
        Eigen::Vector3d pos, vel, omega;
        Eigen::Matrix3d rot;
        bool has_data;
    };
    Snap snap() {
        std::lock_guard<std::mutex> lg(mtx);
        return {pos, vel, omega, rot, has_data};
    }
};

// ── Aggregate display state (set up once in main, then written by trackers) ──
struct DisplayState {
    // Metadata — written once before TUI starts, read-only after
    std::string base_label, rover_label;
    std::string base_key, rover_key, rel_key, gt_key;
    std::string meas_type_str = "rover2base";   // "rover2base" | "base2rover"
    double base_cfg_lat_ms  = 0.0;
    double rover_cfg_lat_ms = 0.0;
    double rel_cfg_lat_ms   = 0.0;
    bool   gt_enabled       = false;

    // Live pose data — written by tracker loops, read by display thread
    PoseSnap base_pose;
    PoseSnap rover_pose;
    PoseSnap rel_pose;
    GTSnap   gt_state;

    // Frequency meters — ticked on every zenoh publish
    FreqMeter base_freq;
    FreqMeter rover_freq;
    FreqMeter rel_freq;
    FreqMeter gt_freq;
};

// ── TUI renderer ─────────────────────────────────────────────────────────────
class TUIDisplay {
public:
    explicit TUIDisplay(DisplayState& ds) : ds_(ds) {}
    ~TUIDisplay() { stop(); }

    void run();                                              // blocking; call from dedicated thread
    void stop() { running_.store(false, std::memory_order_relaxed); }

private:
    DisplayState&        ds_;
    std::atomic<bool>    running_{true};

    void render();
};
