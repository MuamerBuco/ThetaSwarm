#include "ArucoTracking/arucoDetection.h"
#include "MotionControl/feedback.h"

//#include <eigen-3.3.9/Eigen/Dense>

#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    std::thread aruco_thread(start_aruco_detection);
    msDelay(1000);
    std::thread control_thread(run_feedback_control);

    //std::cout << "we got to the next part" << std::endl;

    while(1){
        //runManualControl();
    }
    
    aruco_thread.join();
    control_thread.join();

    //msDelay(5000);
}