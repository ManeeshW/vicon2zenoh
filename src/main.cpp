#include "vicon2pose.hpp"
#include <iostream>
#include <csignal>

bool SYS_ON = true;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        SYS_ON = false;
    }
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signal_handler);
    std::cout << "Starting VICON2POSE program..." << std::endl;

    vicon2pose v2p;
    v2p.open();
    while (SYS_ON) {
        v2p.loop();
    }
    v2p.close();

    std::cout << "Program closed!" << std::endl;
    return 0;
}