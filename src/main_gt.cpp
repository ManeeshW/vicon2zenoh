#include "vicon2gt.hpp"
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
    std::cout << "Starting VICON2GT program..." << std::endl;

    vicon2gt v2gt;
    v2gt.open();
    while (SYS_ON) {
        v2gt.loop();
    }
    v2gt.close();

    std::cout << "Program closed!" << std::endl;
    return 0;
}