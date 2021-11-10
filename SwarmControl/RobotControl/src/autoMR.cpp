#include "../autoMR.h"

#include <cmath>
#include <vector>
#include <fstream>

using namespace Eigen;

#define MAX_FAIL_COUNT 20

// HARDWARE TESTING
uint8_t DIRECTION_ARRAY[6][8] = {
    {255, 1 ,255, 1 ,255, 1 ,255, 1}, // Move Forward
    {255, 0 ,255, 0 ,255, 0 ,255, 0}, // Move Backward

    {255, 1 ,255, 0 ,255, 1 ,255, 0}, //Rotate Right
    {255, 0 ,255, 1 ,255, 0 ,255, 1}, // Rotate Left

    {255, 1 ,255, 0 ,255, 0 ,255, 1}, //Move Right
    {255, 0 ,255, 1 ,255, 1 ,255, 0} // Move Left
};

std::string configFile = "../config.json";

// explicitly initialize robot PD coefficients, HOR matrix and initial states
bool autoMR::initializeRobot(int id)
{
    try {
        loadConfig(configFile, id);
        setFieldData();

        robot_data.kinematics_data.H0_R = initialize_H_0_R(robot_data.robot_configuration);

        initializeRobotStates(id);

        // initialize UDP client
        robot_client = std::make_shared<udp_client_server::udp_client>(robot_data.robotIP, robot_data.robotPort);
    }
    catch( const int& err ) {
        std::cerr << "Failed to load robot config!" << std::endl;
        throw err;
    }
    catch(const std::bad_alloc& e) {
        std::cout << "Allocation of UDP client failed: " << e.what() << ". Aborting... " << std::endl;
        std::exit( EXIT_FAILURE );
    }

    return 1;
}

// grab the robot configuration data from the config file
bool autoMR::loadConfig(std::string filePath, int id)
{
    using namespace rapidjson;

    std::ifstream ifs(filePath);
    if ( !ifs.is_open() )
    {
        std::cerr << "Could not open config file for reading" << std::endl;
        throw FATAL_ERROR;
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

                const Value& robot_PID = robot_json[i]["PID"];
                robot_data.kinematics_data.pd_coefficients.Kp_yaw = robot_PID["Yaw"].GetFloat();
                robot_data.kinematics_data.pd_coefficients.Kp_x = robot_PID["X"].GetFloat();
                robot_data.kinematics_data.pd_coefficients.Kp_y = robot_PID["Y"].GetFloat();
                
                const Value& robot_precision = robot_json[i]["PrecisionMargins"];
                robot_data.robot_constraints.Target_Precision_Margin_Yaw = robot_precision["Yaw"].GetFloat();
                robot_data.robot_constraints.Target_Precision_Margin_X = robot_precision["X"].GetFloat();
                robot_data.robot_constraints.Target_Precision_Margin_Y = robot_precision["Y"].GetFloat();

                const Value& robot_wheel_to_center = robot_json[i]["Wheel_To_Ceter_mm"];
                robot_data.robot_configuration.Wheel_To_CenterX_mm = robot_wheel_to_center["X"].GetFloat();
                robot_data.robot_configuration.Wheel_To_CenterY_mm = robot_wheel_to_center["Y"].GetFloat();

                const Value& robot_viable_pwm = robot_json[i]["Viable_PWM"];
                robot_data.robot_configuration.Max_Viable_PWM = robot_viable_pwm["MAX"].GetInt();
                robot_data.robot_configuration.Min_Viable_PWM = robot_viable_pwm["MIN"].GetInt();
                
                const Value& robot_viable_speed = robot_json[i]["Viable_RadS_Speed"];
                robot_data.robot_configuration.Max_RadS_Speed = robot_viable_speed["MAX"].GetFloat();
                robot_data.robot_configuration.Min_RadS_Speed = robot_viable_speed["MIN"].GetFloat();

                const Value& default_color = robot_json[i]["DefaultColor"];
                robot_data.default_color.r = default_color["Red"].GetInt();
                robot_data.default_color.g = default_color["Green"].GetInt();
                robot_data.default_color.b = default_color["Blue"].GetInt();

                const Value& bucket_constraints = robot_json[i]["BucketConstraints"];
                robot_data.robot_configuration.Max_Bucket_Tilt = bucket_constraints["MaxTilt"].GetFloat();
                robot_data.robot_configuration.Min_Bucket_Tilt = bucket_constraints["MinTilt"].GetFloat();
                robot_data.robot_configuration.Max_Bucket_Extend = bucket_constraints["MaxExtend"].GetFloat();
                robot_data.robot_configuration.Min_Bucket_Extend = bucket_constraints["MinExtend"].GetFloat();

                return 1;
            }
        }

        ifs.close();
        std::cerr << "ID missing from config file" << std::endl;
        throw CASE_ERROR;
    }
    else {
        ifs.close();
        std::cerr << "Config doesn't have Robot member" << std::endl;
        throw FATAL_ERROR;
    }

    return 0;
}

int autoMR::setTrajectory(FullStateTrajectory const &full_trajectory)
{
    if( TrajectorySet->try_push(full_trajectory) ) return 1;
    else return 0;
}

// set initial states to be used
void autoMR::initializeRobotStates(int id)
{
    // current state
    current_full_state.pose_and_id.id = id;
    target_full_state.pose_and_id.id = id;

    // default state
    default_state.pose_and_id.id = id;

    // stop state
    stop_state.pose_and_id.id = id;
}

// get the last recorded robot pose
const ChassisFullState autoMR::getLastRobotPose()
{
    return current_full_state.pose_and_id;
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

// get robot configuration
RobotConfiguration autoMR::getRobotConfig()
{
    return robot_data.robot_configuration;
}

// push the new full robot state state to queue that robot thread reads from
bool autoMR::pushNewRobotState(FullRobotState const  &new_full_robot_state)
{
    return LatestRobotState->try_push(new_full_robot_state);
}

// stop all movement untill the stopRobot flag gets reset
void autoMR::freezeRobot(std::shared_ptr<udp_client_server::udp_client> client_object)
{
    uint8_t shutdown[12] = {0};
    shutdown[0] = 1;

    stopRobot = true;

    while(stopRobot){
        msDelay(10);
        SendRobotCommands(&shutdown[0], client_object, 1);
    }
}

/* **
* run in a preset direction, at set speed, for set duration
* 0 => Move Forward
* 1 => Move Backward
* 2 => Rotate Right
* 3 => Rotate Left
* 4 => Move Right
* 5 => Move Left ** */
void autoMR::setCustomDirection(uint8_t direction, uint8_t speed, int duration)
{
    uint8_t tx_buffer[9] = {0};

    tx_buffer[0] = CUSTOM_MOVE;

    for (int i = 0; i < 8; i++) 
    {
        tx_buffer[i+1] = *(DIRECTION_ARRAY[direction] + i);
        if(i % 2 == 0) tx_buffer[i+1] = speed;
    }

    SendRobotCommandsForMs(&tx_buffer[0], duration, 2, robot_client);
}

/* **
* set custom program, or a single LED
* If mode == 0(SET_SINGLE_PIXEL) use index and colors(0-255)
* If mode == (SET_SINGLE_PIXEL) use colors(0-255)
* If mode == program that uses timing(Rainbow, Theatre) use ms_delay */
void autoMR::setCustomColor(uint8_t index, RGBColor my_color, uint8_t mode, uint8_t ms_delay)
{
    uint8_t tx_buffer[7];

    tx_buffer[0] = CUSTOM_LED;

    tx_buffer[1] = mode;
    tx_buffer[2] = index;

    tx_buffer[3] = my_color.r;
    tx_buffer[4] = my_color.g;
    tx_buffer[5] = my_color.b;

    tx_buffer[6] = ms_delay;

    // resend a few times
    for(int i = 0; i < 2; i ++)
    {
        SendRobotCommands(&tx_buffer[0], robot_client, 1);
    }
}

// set custom tilt and extension of bucket
void autoMR::setCustomBucket(uint8_t tilt, uint8_t extend)
{
    uint8_t tx_buffer[3];

    tx_buffer[0] = CUSTOM_BUCKET;

    tx_buffer[1] = tilt;
    tx_buffer[2] = extend;

    // resend a few times
    for(int i = 0; i < 5; i ++)
    {
        SendRobotCommands(&tx_buffer[0], robot_client, 1);
    }
}

// light up the robot in its default color
void autoMR::selfIdentify()
{
    this->setCustomColor(0, robot_data.default_color, BLINK_ONCE, 0);
}

/* checks each pose parameter
* if any parameter is within bounds of precision margin remove it from error calculations
* if any is outside the margins of precision return false
* if all parameters are within margins, return true */
bool autoMR::ReachedTarget()
{
    if( abs(current_full_state.pose_and_id.pose_state.q.yaw - target_full_state.pose_and_id.pose_state.q.yaw) <= robot_data.robot_constraints.Target_Precision_Margin_Yaw )
    {
        // std::cout << "Im happy with Yaw" << std::endl;
    }
    else return 0;

    if( abs(current_full_state.pose_and_id.pose_state.q.x - target_full_state.pose_and_id.pose_state.q.x) <= robot_data.robot_constraints.Target_Precision_Margin_X )
    {
        // std::cout << "Im happy with X" << std::endl;
    }
    else return 0;

    if( abs(current_full_state.pose_and_id.pose_state.q.y - target_full_state.pose_and_id.pose_state.q.y) <= robot_data.robot_constraints.Target_Precision_Margin_Y )
    {
        // std::cout << "Im happy with Y" << std::endl;
    }
    else return 0;
        
    return 1;
}

// gets the next trajectory from queue and saves it to full_state_trajectory, non-blocking
int autoMR::getNextTrajectory()
{
    if( TrajectorySet->front() )
    {
        full_state_trajectory = *( TrajectorySet->front() );
        TrajectorySet->pop();
        return 1;
    }
    else {
        return 0;
    }
}

// updates current_full_state based on the full_state_trajectory, returns false if full state trajectory is empty
int autoMR::updateFullTrajectory()
{
    if( !full_state_trajectory.empty() )
    {
        SingleStateTrajectory temp_single_state = full_state_trajectory.back();
        full_state_trajectory.pop_back();

        target_full_state.pose_and_id.pose_state.q.x = temp_single_state.pose.x;
        target_full_state.pose_and_id.pose_state.q.y = temp_single_state.pose.y;
        target_full_state.pose_and_id.pose_state.q.yaw = temp_single_state.pose.yaw;
        
        current_full_state.bucket_state = temp_single_state.bucket_state;
        current_full_state.LED_state = temp_single_state.LED_state;

        return 1;
    }
    else return 0;
}

// set next point if available, try to load new trajectory if not, and then set the next point
int autoMR::setNextTrajectoryPoint()
{
    if( updateFullTrajectory() )
    {
        std::cout << "updated full trajectory" << std::endl;
        return 1;
    }
    else {
        if( getNextTrajectory() )
        {
            std::cout << "got new trajectory" << std::endl;
            if( updateFullTrajectory() ) return 1;
            else return 0;
        }
        else {
            // std::cout << "No new trajectory setpoints" << std::endl;
            return 0;
        }
    }
}

Vector3f autoMR::getPoseError()
{
    Vector3f pose_error;

    // calculate error between target and current
    pose_error(0,0) = target_full_state.pose_and_id.pose_state.q.yaw - current_full_state.pose_and_id.pose_state.q.yaw; 
    pose_error(1,0) = target_full_state.pose_and_id.pose_state.q.x - current_full_state.pose_and_id.pose_state.q.x;
    pose_error(2,0) = target_full_state.pose_and_id.pose_state.q.y - current_full_state.pose_and_id.pose_state.q.y;

    // clamp to 0 if within margins to avoid destabilizing
    if( abs(pose_error(0,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_Yaw ) { pose_error(0,0) = 0; }
    if( abs(pose_error(1,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_X ) { pose_error(1,0) = 0; }
    if( abs(pose_error(2,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_Y ) { pose_error(2,0) = 0; }

    return pose_error;
}

// function that the thread runs for robot control
void autoMR::robot_control()
{    
    FullRobotState state_holder;
    Vector3f pose_error;
    Vector3f q_dot;
    float currentPhi = 0;
    uint8_t robot_command_array[12] = {0};

    // TESTING FEEDBACK
    target_full_state.pose_and_id.pose_state.q.yaw = 1;
    target_full_state.pose_and_id.pose_state.q.x = 90;
    target_full_state.pose_and_id.pose_state.q.y = 65;
    // target_full_state.bucket_state.tilt = 30;
    // target_full_state.bucket_state.extension = 30;
    // target_full_state.LED_state.program = SOLID_GREEN;

    while(1) {

        while( !stopRobot )
        {
            // update to latest state
            if( updateCurrentFullRobotState() ) 
            {
                // check if the current setpoint is reached, if yes set a new one
                if( ReachedTarget() )
                {
                    std::cout << "Reached target!!" << std::endl;
                    if( setNextTrajectoryPoint() )
                    {
                        std::cout << "got new trajecotory" << std::endl;
                        /// if we find the next point
                    }
                    else {
                        std::cout << "No more trajectory points set" << std::endl;
                        break; // TODO1 check where this break lands you, probably wrong
                    }
                    ///// if no more points to follow
                }

                pose_error = getPoseError();

                std::cout << "Current YAW: " << current_full_state.pose_and_id.pose_state.q.yaw << std::endl;
                std::cout << "Current X: " << current_full_state.pose_and_id.pose_state.q.x << std::endl;
                std::cout << "Current Y: " << current_full_state.pose_and_id.pose_state.q.y << std::endl;

                q_dot = PD_Controller(pose_error);

                q_dot(0,0) = 0;

                std::cout << "The pose error vector: \n" << q_dot << std::endl;

                currentPhi = current_full_state.pose_and_id.pose_state.q.yaw;

                // std::cout << "The current passed phi: " << currentPhi << std::endl;

                // add first parse bit, leave as 1 for now
                robot_command_array[0] = STANDARD_MODE;

                // TODO2 create a parsing model for engineering and getter functionality
                VectorXi motor_commands = CalculateSpeedCommands(robot_data.kinematics_data.H0_R, robot_data.robot_configuration, q_dot, currentPhi);
            
                // TODO1 switch to smth more elegant
                for(int i = 0; i < 8; i++)
                {
                    robot_command_array[i + 1] = motor_commands(i);
                }

                // robot_command_array[9] = target_full_state.bucket_state.extension;
                // robot_command_array[10] = target_full_state.bucket_state.tilt;
                // robot_command_array[11] = target_full_state.LED_state.program;

                robot_command_array[9] = current_full_state.bucket_state.extension;
                robot_command_array[10] = current_full_state.bucket_state.tilt;
                robot_command_array[11] = current_full_state.LED_state.program;

                // PrintBuffer(&robot_command_array[0]);

                SendRobotCommands(&robot_command_array[0], robot_client, 1); // TODO1 maybe remove 1ms delay
            }
        }
        
        freezeRobot( robot_client );
    }
}

// Directly control robot by passing a q_dot vector [yaw, x, y]
void autoMR::direct_control(Vector3f q_dot)
{
    float currentPhi = 0;
    uint8_t robot_command_array[12] = {0};

    if( updateCurrentFullRobotState() )
    {
        currentPhi = current_full_state.pose_and_id.pose_state.q.yaw;

        // add first parse bit, leave as 1 for now
        robot_command_array[0] = STANDARD_MODE;

        VectorXi motor_commands = CalculateSpeedCommands(robot_data.kinematics_data.H0_R, robot_data.robot_configuration, q_dot, currentPhi);

        for(int i = 0; i < 8; i++)
        {
            robot_command_array[i + 1] = motor_commands(i);
        }

        robot_command_array[9] = target_full_state.bucket_state.extension;
        robot_command_array[10] = target_full_state.bucket_state.tilt;
        robot_command_array[11] = target_full_state.LED_state.program;

        SendRobotCommands(&robot_command_array[0], robot_client, 1);
    }
    else {
        freezeRobot( robot_client );
    }
}

// set what it means for the robot to go into default state
void autoMR::setDefaultState(FullRobotState const &new_default_state)
{
    default_state = new_default_state;
}

// get the field data loaded by aruco
void autoMR::setFieldData()
{
    try{
        robot_data.robot_field_data = getFieldData();
    }
    catch(int& err){
        std::cerr << "Failed to get field data" << std::endl;
        throw err;
    }
    
}

void autoMR::resumeOperation()
{
    stopRobot = false;
}

// force the robot into default state
void autoMR::resetRobot()
{
    pushNewRobotState(default_state);
    stopRobot = false;
}

// immediatelly stop the robot from moving
void autoMR::PANIC_STOP()
{
    stopRobot = true;
}

// map different possible ranges of each variable in the control vector to the same range
Vector3f autoMR::ScaleToEqualRange(Vector3f control_input)
{
    control_input(0,0) = MapValueToRange( robot_data.robot_field_data.Min_Output_Value_P, MIN_NORM_SPEED, robot_data.robot_field_data.Max_Output_Value_P, MAX_NORM_SPEED, control_input(0,0) );
    control_input(1,0) = MapValueToRange( -robot_data.robot_field_data.Max_Output_Value_X, MIN_NORM_SPEED, robot_data.robot_field_data.Max_Output_Value_X, MAX_NORM_SPEED, control_input(1,0) );
    control_input(2,0) = MapValueToRange( -robot_data.robot_field_data.Max_Output_Value_Y, MIN_NORM_SPEED, robot_data.robot_field_data.Max_Output_Value_Y, MAX_NORM_SPEED, control_input(2,0) );

    return control_input;
}

// for now implement only Proportional controller
// TODO2 implement derivative control
Vector3f autoMR::PD_Controller(Vector3f const &pose_error)
{
    Vector3f control_input = ScaleToEqualRange(pose_error);

    // TODO2 introduce speed control as ---- control_input(n, 0) * speed_n ---- after data becomes available 
    control_input(0,0) = robot_data.kinematics_data.pd_coefficients.Kp_yaw * control_input(0,0);
    control_input(1,0) = robot_data.kinematics_data.pd_coefficients.Kp_x * control_input(1,0);
    control_input(2,0) = robot_data.kinematics_data.pd_coefficients.Kp_y * control_input(2,0);

    return control_input;
}

// retrives the newest full robot state if available and saves it in current_full_state, skips if unavailable
int autoMR::updateCurrentFullRobotState()
{
    if( LatestRobotState->front() )
    {
        current_full_state = *LatestRobotState->front();
        LatestRobotState->pop();
        return 1;
    }
    else return 0;
}

void autoMR::getBatteryStatus()
{
    float battery_status = 1;
    robot_data.battery_percentage = battery_status;
}
