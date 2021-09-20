// #include "ArucoTracking/arucoDetection.h"
#include "SwarmControl/swarm.h"

#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    std::thread aruco_thread(start_aruco_detection);

    std::vector<int> usedIDs = {10,15};

    std::cout << "before starting swarm" << std::endl;
    Swarm test_swarm(usedIDs);
    std::cout << "starting swarm" << std::endl;
    
    while(1)
    {
        for(int i = 0; i < 500; i++)
        {
            test_swarm.UpdateSwarm();
        }
    }
    
    aruco_thread.join();

}