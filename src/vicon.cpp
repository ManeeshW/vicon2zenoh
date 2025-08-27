#include "vicon.hpp"
#include <fstream>
#include <iostream>

vicon::vicon() : tracker(nullptr), x_v(Eigen::Vector3d::Zero()), R_vm(Eigen::Matrix3d::Identity()) {
    loadConfig("../config.cfg");
}

vicon::~vicon() {
    close();
}

void vicon::open() {
    open(object);
}

void vicon::open(std::string object_name) {
    tracker = new vrpn_Tracker_Remote(object_name.c_str());
    tracker->register_change_handler(this, vicon::callback);
    on = true;
    std::cout << "VICON: Tracker opened for " << object_name << std::endl;
}

void vicon::close() {
    if (tracker) {
        tracker->unregister_change_handler(this, vicon::callback);
        delete tracker;
        tracker = nullptr;
    }
    on = false;
    std::cout << "VICON: Closed" << std::endl;
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> vicon::loop() {
    if (!on) {
        return {x_v, R_vm};
    }
    tracker->mainloop();
    return {x_v, R_vm};
}

bool vicon::loadConfig(const std::string& configFile) {
    std::ifstream file(configFile);
    if (!file.is_open()) {
        std::cerr << "VICON: Could not open config file: " << configFile << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.empty() || line.find("object:") == std::string::npos) {
            continue;
        }

        size_t pos = line.find("object:");
        if (pos != std::string::npos) {
            object = line.substr(pos + 7);
            object.erase(0, object.find_first_not_of(" \t\""));
            object.erase(object.find_last_not_of(" \t\"") + 1);
            break;
        }
    }

    file.close();
    if (object.empty()) {
        std::cerr << "VICON: No valid object found in config file" << std::endl;
        return false;
    }

    std::cout << "VICON: Loaded object: " << object << std::endl;
    return true;
}

void vicon::callback(void* userdata, const vrpn_TRACKERCB tdata) {
    vicon* self = static_cast<vicon*>(userdata);

    self->x_v(0) = tdata.pos[0];
    self->x_v(1) = tdata.pos[1];
    self->x_v(2) = tdata.pos[2];

    self->R_vm(0, 0) = 1 - 2 * (tdata.quat[1] * tdata.quat[1]) - 2 * (tdata.quat[2] * tdata.quat[2]);
    self->R_vm(0, 1) = 2 * tdata.quat[0] * tdata.quat[1] - 2 * tdata.quat[3] * tdata.quat[2];
    self->R_vm(0, 2) = 2 * tdata.quat[0] * tdata.quat[2] + 2 * tdata.quat[3] * tdata.quat[1];
    self->R_vm(1, 0) = 2 * tdata.quat[0] * tdata.quat[1] + 2 * tdata.quat[3] * tdata.quat[2];
    self->R_vm(1, 1) = 1 - 2 * (tdata.quat[0] * tdata.quat[0]) - 2 * (tdata.quat[2] * tdata.quat[2]);
    self->R_vm(1, 2) = 2 * tdata.quat[1] * tdata.quat[2] - 2 * tdata.quat[3] * tdata.quat[0];
    self->R_vm(2, 0) = 2 * tdata.quat[0] * tdata.quat[2] - 2 * tdata.quat[3] * tdata.quat[1];
    self->R_vm(2, 1) = 2 * tdata.quat[0] * tdata.quat[3] + 2 * tdata.quat[2] * tdata.quat[1];
    self->R_vm(2, 2) = 1 - 2 * (tdata.quat[0] * tdata.quat[0]) - 2 * (tdata.quat[1] * tdata.quat[1]);

    std::cout << "\nx_v: " << self->x_v.transpose() << std::endl;
    std::cout << "R_vm: " << self->R_vm << std::endl;
}