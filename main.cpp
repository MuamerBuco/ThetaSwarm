#include "SwarmControl/swarm.h"

#include <iostream>
#include <chrono>
#include <thread>

void run_hardware_test()
{
    std::vector<int> usedIDs = {11, 13};
    Swarm test_swarm(usedIDs);

    msDelay(1000);

    test_swarm.testSwarmHardware();
}

int main(int argc, char** argv)
{
    std::thread aruco_thread(start_aruco_detection);
    msDelay(300);

    std::vector<int> usedIDs = { 11, 13};

    Swarm test_swarm(usedIDs);

    // run_hardware_test();
    
    while(1)
    {
        for(int i = 0; i < 500; i++)
        {
            test_swarm.UpdateSwarm();
        }
    }

    aruco_thread.join();
}