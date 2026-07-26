#include "vicon2pose.hpp"
#include "relative_pose.hpp"
#include "display.hpp"
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

static bool SYS_ON = true;

void signal_handler(int signal) {
    if (signal == SIGINT) SYS_ON = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    // Parse --no-tui flag
    bool tui_enabled = true;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--no-tui") tui_enabled = false;
    }

    const std::string cfg = "../config.cfg";

    // ── Base tracker ─────────────────────────────────────────────────────────
    vicon2pose base_v2p;
    base_v2p.load_config(cfg, "Base");

    // ── Rover tracker ────────────────────────────────────────────────────────
    vicon2pose rover_v2p;
    rover_v2p.load_config(cfg, "Rover");

    // ── Relative pose publisher ───────────────────────────────────────────────
    RelativePose rel_pose(base_v2p, rover_v2p);
    rel_pose.load_config(cfg);

    // Open enabled trackers
    if (base_v2p.on)  base_v2p.open();
    if (rover_v2p.on) rover_v2p.open();
    if (rel_pose.on)  rel_pose.open();

    // ── Display setup ─────────────────────────────────────────────────────────
    DisplayState ds;

    ds.base_label  = base_v2p.object_name;
    ds.rover_label = rover_v2p.object_name;
    ds.base_key    = base_v2p.zenoh_key;
    ds.rover_key   = rover_v2p.zenoh_key;
    ds.rel_key     = rel_pose.zenoh_key;
    ds.gt_key      = rel_pose.gt_state_key;
    ds.meas_type_str    = rel_pose.meas_type_str();
    ds.base_cfg_lat_ms  = base_v2p.latency  * 1000.0;
    ds.rover_cfg_lat_ms = rover_v2p.latency * 1000.0;
    ds.rel_cfg_lat_ms   = rel_pose.latency  * 1000.0;
    ds.gt_enabled       = rel_pose.gt_state_enable;

    // Wire up display pointers (only if TUI is active)
    TUIDisplay* tui = nullptr;
    std::thread display_thread;

    if (tui_enabled) {
        if (base_v2p.on) {
            base_v2p.display_snap = &ds.base_pose;
            base_v2p.display_freq = &ds.base_freq;
        }
        if (rover_v2p.on) {
            rover_v2p.display_snap = &ds.rover_pose;
            rover_v2p.display_freq = &ds.rover_freq;
        }
        if (rel_pose.on) {
            rel_pose.display_rel_snap = &ds.rel_pose;
            rel_pose.display_rel_freq = &ds.rel_freq;
            rel_pose.display_gt_snap  = &ds.gt_state;
            rel_pose.display_gt_freq  = &ds.gt_freq;
        }

        tui = new TUIDisplay(ds);
        display_thread = std::thread([&]() { tui->run(); });
    } else {
        std::cout << "Starting vicon2zenoh (console mode)... Press Ctrl+C to stop." << std::endl;
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    // Polled at ~100us instead of 1ms so 200Hz deadlines (5ms period) are checked
    // with much tighter granularity — a 1ms poll interval alone eats ~20% of that
    // budget in scheduling slack, which is what was capping achieved rate to ~173Hz.
    while (SYS_ON) {
        if (base_v2p.on)  base_v2p.loop();
        if (rover_v2p.on) rover_v2p.loop();
        if (rel_pose.on)  rel_pose.loop();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // ── Shutdown ──────────────────────────────────────────────────────────────
    if (tui) {
        tui->stop();
        if (display_thread.joinable()) display_thread.join();
        delete tui;
    }

    if (base_v2p.on)  base_v2p.close();
    if (rover_v2p.on) rover_v2p.close();
    if (rel_pose.on)  rel_pose.close();

    std::cout << "Program closed." << std::endl;
    return 0;
}
