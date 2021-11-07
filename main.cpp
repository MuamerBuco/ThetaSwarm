#include "SwarmControl/swarm.h"

#include <iostream>
#include <chrono>
#include <thread>

void run_hardware_test()
{
    std::vector<int> usedIDs = {15, 16};
    Swarm test_swarm(usedIDs);

    test_swarm.testSwarmHardware();
}

int main(int argc, char** argv)
{
    // std::thread aruco_thread(start_aruco_detection);
    // msDelay(100);

    // std::vector<int> usedIDs = {15, 16};

    // Swarm test_swarm(usedIDs);
    
    // while(1)
    // {
    //     for(int i = 0; i < 500; i++)
    //     {
    //         test_swarm.UpdateSwarm();
    //     }
    // }
    
    // aruco_thread.join();

    ////////// TESTING HARDWARE
    run_hardware_test();
}