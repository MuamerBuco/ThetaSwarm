#include "autoMR.h"

#include <cmath>
#include <vector>

using namespace rigtorp;
using namespace Eigen;

SPSCQueue<FullRobotState> LatestRobotState(2);

// initialize robot PD coefficients, HOR matrix and initial states
bool autoMR::initializeRobot(int id)
{
    initializePDCoefficients();

    initialize_H_0_R();

    initializeRobotStates(id);

    // TODO grab from config
    robotIP = "192.168.1.100";
    robotPort = 3333;

    return 1;
}

// set initial states to be used
void autoMR::initializeRobotStates(int id)
{
    ///////////////// current state
    current_full_state.bucket_state.extension = 0;
    current_full_state.bucket_state.tilt = 0;

    current_full_state.LED_state.program = "none";
    
    current_full_state.current_pose_and_id.id = id;
    current_full_state.current_pose_and_id.x = 1;
    current_full_state.current_pose_and_id.y = 1;
    current_full_state.current_pose_and_id.yaw = 1;

    current_full_state.target_pose_and_id.id = id;
    current_full_state.target_pose_and_id.x = 1;
    current_full_state.target_pose_and_id.y = 1;
    current_full_state.target_pose_and_id.yaw = 1;

    ///////////////////////////// default state
    default_state.bucket_state.extension = 0;
    default_state.bucket_state.tilt = 0;

    default_state.LED_state.program = "none";
    
    default_state.current_pose_and_id.id = id;
    default_state.current_pose_and_id.x = 1;
    default_state.current_pose_and_id.y = 1;
    default_state.current_pose_and_id.yaw = 1;

    default_state.target_pose_and_id.id = id;
    default_state.target_pose_and_id.x = 1;
    default_state.target_pose_and_id.y = 1;
    default_state.target_pose_and_id.yaw = 1;

    //////////////////////////////// stop state
    stop_state.bucket_state.extension = 0;
    stop_state.bucket_state.tilt = 0;

    stop_state.LED_state.program = "none";
    
    stop_state.current_pose_and_id.id = id;
    stop_state.current_pose_and_id.x = 1;
    stop_state.current_pose_and_id.y = 1;
    stop_state.current_pose_and_id.yaw = 1;

    stop_state.target_pose_and_id.id = id;
    stop_state.target_pose_and_id.x = 1;
    stop_state.target_pose_and_id.y = 1;
    stop_state.target_pose_and_id.yaw = 1;
}

// get the last recorded robot pose
const ChassisPoseAndID autoMR::getLastRobotPose()
{
    return current_full_state.current_pose_and_id;
}

// get the last recorded LED state
const SignalLED autoMR::getLastLEDState()
{
    return current_full_state.LED_state;
}

// get the last recorded bucket state
const BucketState autoMR::getLastBucketState()
{
    return current_full_state.bucket_state;
}

// push the new state to queue that thread reads from
const bool autoMR::pushStateToThread(FullRobotState new_full_robot_state)
{
    return LatestRobotState.try_push(new_full_robot_state);
}

/* checks each pose parameter
* if any parameter is within bounds of precision margin remove it from error calculations
* if any is outside the margins of precision return false
* if all parameters are within margins, return true */
bool autoMR::ReachedTarget()
{
    if( abs(current_full_state.current_pose_and_id.yaw - current_full_state.target_pose_and_id.yaw) <= PRECISION_MARGIN_YAW )
    {
        current_full_state.current_pose_and_id.yaw = current_full_state.target_pose_and_id.yaw; 
    }
    else return 0;

    if( abs(current_full_state.current_pose_and_id.x - current_full_state.target_pose_and_id.x) <= PRECISION_MARGIN_X )
    {
        current_full_state.current_pose_and_id.x = current_full_state.target_pose_and_id.x;
    }
    else return 0;

    if( abs(current_full_state.current_pose_and_id.y - current_full_state.target_pose_and_id.y) <= PRECISION_MARGIN_Y )
    {
        current_full_state.current_pose_and_id.y = current_full_state.target_pose_and_id.y;
    }
    else return 0;

        
    return 1;
}

// function that the thread runs for robot control
// TODO: implement the robot control
void autoMR::robot_control()
{
    udp_client robot_client(robotIP, robotPort);
    
    FullRobotState state_holder;
    Vector3f pose_error;
    Vector3f q_dot;
    int currentPhi;
    uint8_t speeds_and_directions[9] = {0};

    while(1)
    {
        while( !ReachedTarget() )
        {
            state_holder = getCurrentFullRobotState();

            pose_error(0,0) = current_full_state.current_pose_and_id.yaw - current_full_state.target_pose_and_id.yaw; 
            pose_error(1,0) = current_full_state.current_pose_and_id.x - current_full_state.target_pose_and_id.x;
            pose_error(2,0) = current_full_state.current_pose_and_id.y - current_full_state.target_pose_and_id.y;

            q_dot = PD_Controller(pose_error);

            // TODO scale yaw into the proper value range here or in arucoDetection, depending on the format requirements
            currentPhi = current_full_state.current_pose_and_id.yaw;
        
            if( CalculateMotorSpeedVector(q_dot, &speeds_and_directions[0], currentPhi) )
            {
                SendMotorCommands(&speeds_and_directions[0], &robot_client, 1); // TODO maybe remove 1ms delay
                //PrintBuffer(&speeds_and_directions[0]);
            }
        }

        std::cout << "yaay reached target" << std::endl;
    }
}

// set what it means for the robot to go into default state
void autoMR::setDefaultState(FullRobotState new_default_state)
{
    default_state = new_default_state;
}

// force the robot into default state
void autoMR::resetRobot()
{
    pushStateToThread(autoMR::default_state);
}

// immediatelly stop the robot from moving
void autoMR::PANIC_STOP()
{
    pushStateToThread(autoMR::stop_state);
}

// initialize proportional-derivative control coefficients
// TODO create per robot basis initialization, enter into top lvl config file 
void autoMR::initializePDCoefficients()
{
    robot_data.pd_coefficients.Kp_p = PROPORTIONAL_CONTROL_COEFFICIENT_P;
    robot_data.pd_coefficients.Kp_x = PROPORTIONAL_CONTROL_COEFFICIENT_X;
    robot_data.pd_coefficients.Kp_y = PROPORTIONAL_CONTROL_COEFFICIENT_Y;
}

// for now implement only Proportional controller
// TODO implement derivative control
Vector3f autoMR::PD_Controller(Vector3f pose_error)
{
    Vector3f control_input(3,1);

    control_input = ScaleToEqualRange(pose_error);

    //std::cout << "The after scale input is: " << std::endl << control_input << std::endl;

    control_input(0,0) = robot_data.pd_coefficients.Kp_p * control_input(0,0);
    control_input(1,0) = robot_data.pd_coefficients.Kp_x * control_input(1,0);
    control_input(2,0) = robot_data.pd_coefficients.Kp_y * control_input(2,0);

    //std::cout << "The after scale and P input is: " << std::endl << control_input << std::endl;

    return control_input;
}

// retrives the newest full robot state, blocking if empty
// TODO set natural no-command state to stay at rest
FullRobotState autoMR::getCurrentFullRobotState()
{
    while(!LatestRobotState.front());
    FullRobotState state_holder = *LatestRobotState.front();

    LatestRobotState.pop();

    return state_holder;
}

// map different possible ranges of each variable in the control vector to the same range
// TODO make the input and output range settable/dynamic
Vector3f ScaleToEqualRange(Vector3f control_input)
{
    control_input(0,0) = MapValueToRange( -180, -100, 180, 100, control_input(0,0) );
    control_input(1,0) = MapValueToRange( -190, -100, 190, 100, control_input(1,0) );
    control_input(2,0) = MapValueToRange( -120, 100, 120, -100, control_input(2,0) );

    return control_input;
}