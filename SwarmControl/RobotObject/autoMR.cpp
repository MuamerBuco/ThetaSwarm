#include "autoMR.h"

#include <cmath>
#include <vector>

using namespace Eigen;

// queues the latest robot data(current pose, target pose, bucket state...)
rigtorp::SPSCQueue<FullRobotState> LatestRobotState(2);

std::string configFile = "../../config.json";

// explicitly initialize robot PD coefficients, HOR matrix and initial states
bool autoMR::initializeRobot(int id)
{
    loadConfig(configFile, id);

    robot_data.kinematics_data.H0_R = initialize_H_0_R(robot_data.robot_configuration);

    initializeRobotStates(id);

    return 1;
}

// grab the robot configuration data from the config file
void autoMR::loadConfig(std::string filePath, int id)
{
    using namespace rapidjson;

    std::ifstream ifs(filePath);
    if ( !ifs.is_open() )
    {
        std::cerr << "Could not open config file for reading!\n";
    }

    IStreamWrapper isw(ifs);
    Document document;
    document.ParseStream(isw);

    assert(document.IsObject());

    if (document.HasMember("Robots"))
    {
        const Value& robot_json = document["Robots"];
        for (SizeType i = 0 ; i < robot_json.Size(); ++i)
        {
            if( robot_json[i]["ID"].GetInt() == id )
            {
                robot_data.ID = robot_json[i]["ID"].GetInt();
                robot_data.robotIP = robot_json[i]["RobotIP"].GetString();
                robot_data.robotPort = robot_json[i]["RobotPort"].GetInt();
                robot_data.robot_configuration.Wheel_Diameter_mm = robot_json[i]["Wheel_Diameter_mm"].GetFloat();

                const Value& robot_PID = robot_json["PID"];
                robot_data.kinematics_data.pd_coefficients.Kp_yaw = robot_PID["Yaw"].GetFloat();
                robot_data.kinematics_data.pd_coefficients.Kp_yaw = robot_PID["X"].GetFloat();
                robot_data.kinematics_data.pd_coefficients.Kp_yaw = robot_PID["Y"].GetFloat();
                
                const Value& robot_precision = robot_json["PrecisionMargins"];
                robot_data.robot_constraints.Target_Precision_Margin_Yaw = robot_precision["Yaw"].GetFloat();
                robot_data.robot_constraints.Target_Precision_Margin_X = robot_precision["X"].GetFloat();
                robot_data.robot_constraints.Target_Precision_Margin_Y = robot_precision["Y"].GetFloat();

                const Value& robot_wheel_to_center = robot_json["Wheel_To_Ceter_mm"];
                robot_data.robot_configuration.Wheel_To_CenterX_mm = robot_wheel_to_center["X"].GetFloat();
                robot_data.robot_configuration.Wheel_To_CenterY_mm = robot_wheel_to_center["Y"].GetFloat();
                
                const Value& robot_max_rot_speed = robot_json["Max_RadS_RotationSpeed"];
                robot_data.robot_configuration.Max_RadS_RotationSpeed_LU = robot_max_rot_speed["LU"].GetFloat(); 
                robot_data.robot_configuration.Max_RadS_RotationSpeed_RU = robot_max_rot_speed["RU"].GetFloat();
                robot_data.robot_configuration.Max_RadS_RotationSpeed_LD = robot_max_rot_speed["LD"].GetFloat();
                robot_data.robot_configuration.Max_RadS_RotationSpeed_RD = robot_max_rot_speed["RD"].GetFloat();

                const Value& robot_min_rot_speed = robot_json["Min_RadS_RotationSpeed"];
                robot_data.robot_configuration.Min_RadS_RotationSpeed_LU = robot_min_rot_speed["LU"].GetFloat(); 
                robot_data.robot_configuration.Min_RadS_RotationSpeed_RU = robot_min_rot_speed["RU"].GetFloat();
                robot_data.robot_configuration.Min_RadS_RotationSpeed_LD = robot_min_rot_speed["LD"].GetFloat();
                robot_data.robot_configuration.Min_RadS_RotationSpeed_RD = robot_min_rot_speed["RD"].GetFloat();

                const Value& robot_viable_pwm = robot_json["Viable_PWM"];
                robot_data.robot_configuration.Max_Viable_PWM = robot_viable_pwm["MAX"].GetInt();
                robot_data.robot_configuration.Min_Viable_PWM = robot_viable_pwm["MIN"].GetInt();
                
                const Value& robot_viable_speed = robot_json["Viable_RadS_Speed"];
                robot_data.robot_configuration.Max_RadS_Speed = robot_viable_speed["MAX"].GetFloat();
                robot_data.robot_configuration.Min_RadS_Speed = robot_viable_speed["MIN"].GetFloat();
            }
        }
    }

    ifs.close();
}

// set initial states to be used
void autoMR::initializeRobotStates(int id)
{
    ///////////////// current state
    current_full_state.current_pose_and_id.id = id;

    current_full_state.target_pose_and_id.id = id;

    ///////////////////////////// default state
    default_state.current_pose_and_id.id = id;

    default_state.target_pose_and_id.id = id;

    //////////////////////////////// stop state
    stop_state.current_pose_and_id.id = id;

    stop_state.target_pose_and_id.id = id;
}

// get the last recorded robot pose
const ChassisFullState autoMR::getLastRobotPose()
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
bool autoMR::pushStateToThread(FullRobotState new_full_robot_state)
{
    return LatestRobotState.try_push(new_full_robot_state);
}

/* checks each pose parameter
* if any parameter is within bounds of precision margin remove it from error calculations
* if any is outside the margins of precision return false
* if all parameters are within margins, return true */
bool autoMR::ReachedTarget()
{
    if( abs(current_full_state.current_pose_and_id.pose_state.q.yaw - current_full_state.target_pose_and_id.pose_state.q.yaw) <= robot_data.robot_constraints.Target_Precision_Margin_Yaw)
    {
        current_full_state.current_pose_and_id.pose_state.q.yaw = current_full_state.target_pose_and_id.pose_state.q.yaw; 
    }
    else return 0;

    if( abs(current_full_state.current_pose_and_id.pose_state.q.x - current_full_state.target_pose_and_id.pose_state.q.x) <= robot_data.robot_constraints.Target_Precision_Margin_X )
    {
        current_full_state.current_pose_and_id.pose_state.q.x = current_full_state.target_pose_and_id.pose_state.q.x;
    }
    else return 0;

    if( abs(current_full_state.current_pose_and_id.pose_state.q.y- current_full_state.target_pose_and_id.pose_state.q.y) <= robot_data.robot_constraints.Target_Precision_Margin_Y )
    {
        current_full_state.current_pose_and_id.pose_state.q.y = current_full_state.target_pose_and_id.pose_state.q.y;
    }
    else return 0;

        
    return 1;
}

// function that the thread runs for robot control
// TODO: implement the robot control
void autoMR::robot_control()
{
    udp_client robot_client(robot_data.robotIP, robot_data.robotPort);
    
    FullRobotState state_holder;
    Vector3f pose_error;
    Vector3f q_dot;
    int currentPhi;
    uint8_t speeds_and_directions[9] = {0};

    while(1) //TODO change while forever to something stoppable
    {
        while( !ReachedTarget() )
        {
            // update to latest state
            updateCurrentFullRobotState();

            // calculate error between target and current
            pose_error(0,0) = current_full_state.current_pose_and_id.pose_state.q.yaw - current_full_state.target_pose_and_id.pose_state.q.yaw; 
            pose_error(1,0) = current_full_state.current_pose_and_id.pose_state.q.x - current_full_state.target_pose_and_id.pose_state.q.x;
            pose_error(2,0) = current_full_state.current_pose_and_id.pose_state.q.y - current_full_state.target_pose_and_id.pose_state.q.y;

            q_dot = PD_Controller(pose_error);

            // TODO scale yaw into the proper value range here or in arucoDetection, depending on the format requirements
            currentPhi = current_full_state.current_pose_and_id.pose_state.q.yaw;

            //TODO create a parsing model for engineering and getter functionality
            if( CalculateMotorSpeedVector(robot_data.kinematics_data.H0_R, &robot_data.robot_configuration, q_dot, &speeds_and_directions[0], currentPhi) )
            {
                SendRobotCommands(&speeds_and_directions[0], &robot_client, 1); // TODO maybe remove 1ms delay
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

// map different possible ranges of each variable in the control vector to the same range
// TODO make the input and output range settable/dynamic
Vector3f ScaleToEqualRange(Vector3f control_input)
{
    control_input(0,0) = MapValueToRange( -180, -100, 180, 100, control_input(0,0) );
    control_input(1,0) = MapValueToRange( -190, -100, 190, 100, control_input(1,0) );
    control_input(2,0) = MapValueToRange( -120, 100, 120, -100, control_input(2,0) );

    return control_input;
}

// for now implement only Proportional controller
// TODO implement derivative control
Vector3f autoMR::PD_Controller(Vector3f pose_error)
{
    Vector3f control_input = ScaleToEqualRange(pose_error);

    //std::cout << "The after scale input is: " << std::endl << control_input << std::endl;

    // TODO introduce speed control as ---- control_input(n, 0) * speed_n ---- after data becomes available 
    control_input(0,0) = robot_data.kinematics_data.pd_coefficients.Kp_yaw * control_input(0,0);
    control_input(1,0) = robot_data.kinematics_data.pd_coefficients.Kp_x * control_input(1,0);
    control_input(2,0) = robot_data.kinematics_data.pd_coefficients.Kp_y * control_input(2,0);

    //std::cout << "The after scale and P input is: " << std::endl << control_input << std::endl;

    return control_input;
}

// retrives the newest full robot state if available and saves it in current_full_state, skips if unavailable
// TODO set natural no-command state to stay at rest
void autoMR::updateCurrentFullRobotState()
{
    if( LatestRobotState.front() )
    {
        current_full_state = *LatestRobotState.front();
        LatestRobotState.pop();
    }
    else ; // TODO add clamping, fading or zero default state; add fail counter, after n fails start sending stop commands
}

void autoMR::getBatteryStatus()
{
    float battery_status = 1; // TODO make sendRequestCommand() to get the battery reading from the robot
    robot_data.battery_percentage = battery_status;
}
