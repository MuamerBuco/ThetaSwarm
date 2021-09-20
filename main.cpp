// #include "ArucoTracking/arucoDetection.h"
#include "SwarmControl/swarm.h"

#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    std::thread aruco_thread(start_aruco_detection);

    std::cout << "before starting swarm" << std::endl;
    Swarm test_swarm;
    std::cout << "starting swarm" << std::endl;
    
    while(1)
    {
        // std::cout << "starting main while 1" << std::endl;
        test_swarm.UpdateSwarm();
    }
    
    // aruco_thread.join();

}