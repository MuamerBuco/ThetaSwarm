#include "feedback.h"
#include "../kinematics.h"
#include "../../PathPlanning/navigation.h"
#include "../../../Client/udp_client_server.h"
#include "../../../ArucoTracking/arucoDetection.h"
#include "../../../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"

#include <thread>
#include <cmath>
#include <vector>

using namespace rigtorp;

const std::string ip_addr = "192.168.1.100";
const int port = 3333;

#define PROPORTIONAL_CONTROL_COEFFICIENT_P 1
#define PROPORTIONAL_CONTROL_COEFFICIENT_X 1
#define PROPORTIONAL_CONTROL_COEFFICIENT_Y 1

struct PD_Controller_Coefficients
{
    int Kp_p;
    int Kp_x;
    int Kp_y;
};

// PD Controller coefficients
PD_Controller_Coefficients coefficients1;

void initializePDCoefficients()
{
    coefficients1.Kp_p = PROPORTIONAL_CONTROL_COEFFICIENT_P;
    coefficients1.Kp_x = PROPORTIONAL_CONTROL_COEFFICIENT_X;
    coefficients1.Kp_y = PROPORTIONAL_CONTROL_COEFFICIENT_Y;
}

// take in a {phi_dot, x_dot, y_dot} vector, convert to appropriate speed-direction commands and send
void SendVectorControl(VectorXi control_input, int *current_phi, udp_client *myClient)
{
    uint8_t speeds_and_directions[9] = {0};
    
    if(CalculateMotorSpeedVector(control_input, &speeds_and_directions[0], current_phi)){
        SendMotorCommands(&speeds_and_directions[0], myClient, 1);
        //PrintBuffer(&speeds_and_directions[0]);
    }
}

// map different possible ranges of each variable to the same range
VectorXi ScaleToEqualRange(VectorXi control_input)
{
    control_input(0,0) = MapValueToRange( -180, -100, 180, 100, control_input(0,0) );
    control_input(1,0) = MapValueToRange( -190, -100, 190, 100, control_input(1,0) );
    control_input(2,0) = MapValueToRange( -120, 100, 120, -100, control_input(2,0) );

    return control_input;
}

// for now implement only Proportional controller
VectorXi PD_Controller(VectorXi pose_error)
{
    VectorXi control_input(3,1);

    control_input = ScaleToEqualRange(pose_error);

    //std::cout << "The after scale input is: " << std::endl << control_input << std::endl;

    control_input(0,0) = coefficients1.Kp_p * control_input(0,0);
    control_input(1,0) = coefficients1.Kp_x * control_input(1,0);
    control_input(2,0) = coefficients1.Kp_y * control_input(2,0);

    //std::cout << "The after scale and P input is: " << std::endl << control_input << std::endl;

    // int   var_0 = static_cast<int>(control_input(0,0));
    // int   var_1 = static_cast<int>(control_input(1,0));
    // int   var_2 = static_cast<int>(control_input(2,0));

    // control_input(0,0) = static_cast<float>(var_0);
    // control_input(1,0) = static_cast<float>(var_1);
    // control_input(2,0) = static_cast<float>(var_2);

    return control_input;
}

VectorXi GetCurrentPose()
{
    AllPosesInPass pose_holder = getAllPosesAndIDs();

    VectorXi current_pose(3,1);

    current_pose(0,0) = static_cast<int>(pose_holder.poses[0].yaw);
    current_pose(1,0) = static_cast<int>(pose_holder.poses[0].x);
    current_pose(2,0) = static_cast<int>(pose_holder.poses[0].y);

    return current_pose;
}

// runs in a loop on a separate thread, takes front current poses of robots from buffer
// takes new target poses if available, if not uses old
// calculates error from target
// runs error into Proportional control to get a q_dot control vector
// applies non holonomic constraints on the chassis velocities
// converts chassis velocities to wheel speeds, wheel speeds to pwm direction array
// sends set of wheel commands to robot
void calculate_feedback(std::string ip_addr, int port)
{
    // initialize udp object
    udp_client myClient(ip_addr, port);

    AllTargetPoses target_poses_holder;
    TargetPose single_target_pose;
    
    VectorXi target_pose_vector(3,1);
    VectorXi current_pose_vector(3,1);
    int current_phi;
    
    // Initialize kinematics
    initialize_H_0_R();

    // initialize PD controller parameters
    initializePDCoefficients();

    // prepare a dummy target buffer for testing
    fill_target_pose_buffer();

    auto elapsedTotal = 0;

    while(1) {

        for(int test = 0; test < 1000; test++){
        auto startfb = std::chrono::system_clock::now();
        //fill_target_pose_buffer();
        // if there's a new set point in the buffer, update and pop, if not skip, throw exception and use latest
        try {

            //std::cout << "We got to try, lets throw stuff" << std::endl;
            target_poses_holder = getAllTargetPosesAndIDs();
            single_target_pose = target_poses_holder.allPosesVector[0];

            target_pose_vector(0,0) = static_cast<int>(single_target_pose.p);
            target_pose_vector(1,0) = static_cast<int>(single_target_pose.x);
            target_pose_vector(2,0) = static_cast<int>(single_target_pose.y);

            //std::cout << "target_pose_vector" << target_pose_vector << std::endl;

        }
        catch( const std::invalid_argument& e) {
            // nothing yet
        }
        
        // map tvecs and rvecs to a pose vector in coordinates
        current_pose_vector = GetCurrentPose();
        
        // #testing
        std::cout << "The current phi: " << current_pose_vector(0,0) << std::endl;
        //current_pose_vector(0,0) = 0;
        //current_pose_vector(1,0) = 160;
        //current_pose_vector(2,0) = 100;
        // updates current degree offset from 0 in degrees
        current_phi = current_pose_vector(0,0);

        //current_phi = static_cast<int>( MapValueToRange(-180, 0, 180, 360, current_pose_vector(0,0)) );
        //std::cout << "current_phi new" << current_phi << std::endl;
        // calculate error vector
        VectorXi pose_error_vector = target_pose_vector - current_pose_vector;

        //std::cout << "target_pose_vector" << target_pose_vector << std::endl;

        //std::cout << "current pose" << std::endl << current_pose_vector << std::endl;

        //std::cout << "pose_error_vector" << std::endl << pose_error_vector << std::endl;

        // if(abs(pose_error_vector(0,0)) < 5) std::cout << "0th is < 5" << std::endl;
        // if(abs(pose_error_vector(0,0)) < 5) std::cout << "1st is < 5" << std::endl;
        // if(abs(pose_error_vector(0,0)) < 5) std::cout << "2nd is < 5" << std::endl;

        // if(abs(pose_error_vector(0,0)) < 2 && abs(pose_error_vector(1,0)) < 15 && abs(pose_error_vector(2,0)) < 15) {
        //     std::cout << "gasi struju, motaj kablove" << std::endl;
        //     break;
        // }

        // apply proportional control
        VectorXi q_dot = PD_Controller(pose_error_vector);

        // #testing
        //current_phi = 150;
        q_dot(0,0) = 0;
        q_dot(1,0) = 100;
        q_dot(2,0) = 0;
        SendVectorControl(q_dot, &current_phi, &myClient);

        //std::cout << "The target_pose_vector vector: " << target_pose_vector << std::endl;
        //std::cout << "The q_dot vector: " << std::endl << q_dot << std::endl;
        //std::cout << "The pose_error_vector vector: " << pose_error_vector << std::endl;


        auto endfb = std::chrono::system_clock::now();
        auto elapsedfb = std::chrono::duration_cast<std::chrono::milliseconds>(endfb - startfb).count();
        elapsedTotal += elapsedfb;
        //std::cout << "The feedback pass took: " << 1000/(elapsedfb.count()/1000000) << std::endl;
    
        //msDelay(100);

        }//test 100
        
        std::cout << "The average runthrough time in ms: " << elapsedTotal/1000 << std::endl;
        elapsedTotal = 0;
    }

    std::cout << "We did iiit" << std::endl;
    SendMotorShutdownCommand(&myClient);

    while(1) {
        //
    }
}

void run_feedback_control()
{
    std::thread feedback_thread(calculate_feedback, ip_addr, port);

    feedback_thread.join();
}


