#include "ArucoTracking/arucoDetection.h"

#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    std::thread aruco_thread(start_aruco_detection);
    
    //std::cout << "we got to the next part" << std::endl;

    while(1){
        //runManualControl();
    }
    
    aruco_thread.join();
}