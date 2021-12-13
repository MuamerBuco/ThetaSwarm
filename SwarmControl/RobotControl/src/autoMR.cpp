#include "../autoMR.h"

#include <cmath>
#include <vector>
#include <fstream>

using namespace Eigen;

#define MAX_FAIL_COUNT 20
#define MAX_CONT_INTERPOLATION_STEPS 100

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

// Explicitly initialize robot PD coefficients, HOR matrix and initial states
bool autoMR::initializeRobot(int id)
{
    try {
        loadConfig(configFile, id);
        setFieldData();

        robot_data.kinematics_data.H0_R = initialize_H_0_R(robot_data.robot_configuration);

        initializeRobotStates(id);

        robot_data.robot_configuration.setViablePWM();

        // initialize UDP client
        robot_client = std::make_shared<udp_client_server::udp_client>(robot_data.robotIP, robot_data.robotPort);
    }
    catch( const int& err ) {
        std::cerr << "Failed to load robot config!" << std::endl;
        throw err;
    }
    catch(const std::bad_alloc& e) {
        std::cerr << "Allocation of UDP client failed: " << e.what() << ". Aborting... " << std::endl;
        std::exit( EXIT_FAILURE );
    }

    return 1;
}

// Grab the robot configuration data from the config file
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
                
                const Value& robot_max_error = robot_json[i]["ErrorLimits"];
                robot_data.robot_constraints.Error_Limit_Max_Yaw = robot_max_error["Yaw"].GetFloat();
                robot_data.robot_constraints.Error_Limit_Max_X = robot_max_error["X"].GetFloat();
                robot_data.robot_constraints.Error_Limit_Max_Y = robot_max_error["Y"].GetFloat();

                const Value& robot_wheel_to_center = robot_json[i]["Wheel_To_Ceter_mm"];
                robot_data.robot_configuration.Wheel_To_CenterX_mm = robot_wheel_to_center["X"].GetFloat();
                robot_data.robot_configuration.Wheel_To_CenterY_mm = robot_wheel_to_center["Y"].GetFloat();

                const Value& robot_viable_pwm = robot_json[i]["Viable_PWM"];
                robot_data.robot_configuration.Max_Viable_PWM = robot_viable_pwm["MAX"].GetInt();
                robot_data.robot_configuration.Min_Viable_PWM = robot_viable_pwm["MIN"].GetInt();
                
                // TODO repurpose or remove
                const Value& Controller_Set_Speed = robot_json[i]["Controller_Set_Speed"];
                robot_data.robot_configuration.Max_Controller_Speed = Controller_Set_Speed["MAX"].GetFloat();
                robot_data.robot_configuration.Max_Controller_Speed = Controller_Set_Speed["MIN"].GetFloat();

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

// Set initial states to be used
void autoMR::initializeRobotStates(int id)
{
    current_full_state.pose_and_id.id = id;
    target_full_state.pose_and_id.id = id;

    default_state.pose_and_id.id = id;

    stop_state.pose_and_id.id = id;
}

// Get the last recorded robot pose
const ChassisFullState autoMR::getLastRobotPose()
{
    return current_full_state.pose_and_id;
}

// Get the last recorded LED state
const SignalLED autoMR::getLastLEDState()
{
    return current_full_state.LED_state;
}

// Get the last recorded bucket state
const BucketState autoMR::getLastBucketState()
{
    return current_full_state.bucket_state;
}

// Get robot configuration
RobotConfiguration autoMR::getRobotConfig()
{
    return robot_data.robot_configuration;
}

// Push the new full robot state state to queue that robot thread reads from
bool autoMR::pushNewRobotState(FullRobotState const  &new_full_robot_state)
{
    return LatestRobotState->try_push(new_full_robot_state);
}

// stream motor commands to client object
void autoMR::SendRobotCommands(uint8_t *msgRobot, uint16_t ms_delay)
{
    // PrintBuffer(msgRobot);
    robot_client->send_bytes(msgRobot, sizeof(msgRobot) + 1);
    msDelay(ms_delay);
}

// send set motor commands for [time_in_ms] milliseconds every [send_increment_ms] to client object
void autoMR::SendRobotCommandsForMs(uint8_t *msgRobot, uint16_t time_in_ms, uint16_t send_increment_ms)
{
    uint16_t number_of_cycles = time_in_ms/send_increment_ms;

    for(uint16_t i = 0; i < number_of_cycles; i++) {
        SendRobotCommands(msgRobot, send_increment_ms);
    }
}

// Stop all movement untill the stopRobot flag gets reset
void autoMR::freezeRobot()
{
    uint8_t shutdown[12] = {0};
    shutdown[0] = 1;

    stopRobot = true;

    for(int i = 0; i < 10; i++)
    {
        msDelay(5);
        SendRobotCommands(&shutdown[0], 1);
    }
}

/* **
* Run in a preset direction, at set speed, for set duration
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

    SendRobotCommandsForMs(&tx_buffer[0], duration, 2);
}

/* **
* Set custom program, or a single LED
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
        SendRobotCommands(&tx_buffer[0], 1);
    }
}

// Set custom tilt and extension of bucket
void autoMR::setCustomBucket(uint8_t tilt, uint8_t extend)
{
    uint8_t tx_buffer[3];

    tx_buffer[0] = CUSTOM_BUCKET;

    tx_buffer[1] = tilt;
    tx_buffer[2] = extend;

    // resend a few times
    for(int i = 0; i < 5; i ++)
    {
        SendRobotCommands(&tx_buffer[0], 1);
    }
}

// Light up the robot in its default color
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

// Gets the next trajectory from queue and saves it to full_state_trajectory, non-blocking
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

// Updates current_full_state based on the full_state_trajectory, returns false if full state trajectory is empty
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

// Set next point if available, try to load new trajectory if not, and then set the next point
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

// Calculates pose error, clamps to 0 if lower than margin assigned in config to avoid instability, caps error to maximum assigned in config
Vector3f autoMR::getPoseError()
{
    Vector3f pose_error;
    float solution_1;
    float solution_2;

    float current_yaw = current_full_state.pose_and_id.pose_state.q.yaw;
    float target_yaw = target_full_state.pose_and_id.pose_state.q.yaw;
    float max_error_yaw = robot_data.robot_constraints.Error_Limit_Max_Yaw;

    int target_sign = getSign(target_yaw);
    int current_sign = getSign(current_yaw);

    if(target_sign == current_sign)
    {
        pose_error(0,0) = -target_yaw + current_yaw;
    }
    else 
    {
        if(target_sign > current_sign)
        {
            solution_1 = (max_error_yaw + current_yaw) + (max_error_yaw - target_yaw);
            solution_2 = abs(current_yaw) + target_yaw;
        }
        else {
            solution_1 = (max_error_yaw + target_yaw) + (max_error_yaw - current_yaw);
            solution_2 = abs(target_yaw) + current_yaw;
        }

        if( solution_1 <= solution_2 )
        {
            pose_error(0,0) = solution_1;
        }
        else {
            pose_error(0,0) = -solution_2;
        }
    }


    // calculate error between target and current
    pose_error(1,0) = target_full_state.pose_and_id.pose_state.q.x - current_full_state.pose_and_id.pose_state.q.x;
    pose_error(2,0) = target_full_state.pose_and_id.pose_state.q.y - current_full_state.pose_and_id.pose_state.q.y;

    // Limit to max error set in config
    if( abs(pose_error(1,0)) > robot_data.robot_constraints.Error_Limit_Max_X ) { pose_error(1,0) = robot_data.robot_constraints.Error_Limit_Max_X * getSign(pose_error(1,0)); }
    if( abs(pose_error(2,0)) > robot_data.robot_constraints.Error_Limit_Max_Y ) { pose_error(2,0) = robot_data.robot_constraints.Error_Limit_Max_Y * getSign(pose_error(2,0)); }

    // clamp to 0 if within margins to avoid destabilizing
    if( abs( pose_error(0,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_Yaw ) { pose_error(0,0) = 0; }
    if( abs( pose_error(1,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_X ) { pose_error(1,0) = 0; }
    if( abs( pose_error(2,0) ) <= robot_data.robot_constraints.Target_Precision_Margin_Y ) { pose_error(2,0) = 0; }

    return pose_error;
}

// Function that the thread runs for robot control
void autoMR::robot_control()
{    
    FullRobotState state_holder;
    Vector3f pose_error;
    float currentPhi = 0;
    uint8_t robot_command_array[12] = {0};

    int interpolation_step = 0;

    while(1) {

        while( !stopRobot )
        {
            // update to latest state if ready from aruco tracking, interpolate if not
            if( updateCurrentFullRobotState() ) 
            {
                resetInterpolationTimer();
                interpolation_step = 0;
            }
            else if( interpolateDeadReckoningAndUpdate() ) 
            {
                // if interpolation is not interrupted by real data inflow, stop the robot
                if(interpolation_step > MAX_CONT_INTERPOLATION_STEPS) 
                { 
                    msDelay(3);
                    break;    
                }
                interpolation_step++;
                msDelay(1);
                std::cout << "Just interpolated" << std::endl;
            }
            else break;

            // TESTING FEEDBACK
            // target_full_state.pose_and_id.pose_state.q.yaw = 3.14;//current_full_state.pose_and_id.pose_state.q.yaw;//3.14;
            // target_full_state.pose_and_id.pose_state.q.x = 30;//current_full_state.pose_and_id.pose_state.q.x;//90;
            // target_full_state.pose_and_id.pose_state.q.y = 80;//current_full_state.pose_and_id.pose_state.q.y;//65;
            // target_full_state.bucket_state.tilt = 30;
            // target_full_state.bucket_state.extension = 30;
            // target_full_state.LED_state.program = NONE;

            // calculate error
            pose_error = getPoseError();

            // check if the current setpoint is reached, if yes set a new one
            if( ReachedTarget() )
            {
                std::cout << "Reached target!!" << std::endl;
                freezeRobot();
                if( setNextTrajectoryPoint() )
                {
                    std::cout << "Got new trajectory" << std::endl;
                    /// if we find the next point
                }
                else {
                    std::cout << "No more trajectory points set" << std::endl;
                    msDelay(3);
                    break;
                }
            }

            // std::cout << "Current YAW: " << current_full_state.pose_and_id.pose_state.q.yaw << std::endl;
            // std::cout << "Current X: " << current_full_state.pose_and_id.pose_state.q.x << std::endl;
            // std::cout << "Current Y: " << current_full_state.pose_and_id.pose_state.q.y << std::endl;

            // std::cout << "The pose_error vector: \n" << pose_error << std::endl;

            float speed_coeff = P_Controller(pose_error);

            currentPhi = current_full_state.pose_and_id.pose_state.q.yaw;

            // std::cout << "The current passed phi: " << currentPhi << std::endl;

            robot_command_array[0] = STANDARD_MODE;

            // TODO2 create a parsing model for engineering and getter functionality
            VectorXi motor_commands = CalculateSpeedCommands(robot_data.kinematics_data.H0_R, robot_data.robot_configuration, pose_error, currentPhi, speed_coeff);
        
            for(int i = 0; i < 8; i++)
            {
                robot_command_array[i + 1] = motor_commands(i);
            }

            robot_command_array[9] = target_full_state.bucket_state.extension;
            robot_command_array[10] = target_full_state.bucket_state.tilt;
            robot_command_array[11] = target_full_state.LED_state.program;
            PrintBuffer(&robot_command_array[0]);

            SendRobotCommands(&robot_command_array[0], 1); // TODO1 maybe remove 1ms delay
        }
        msDelay(5); // give other threads some space
    }
    freezeRobot();
}

void autoMR::resetInterpolationTimer()
{
    interpolation_time_start = std::chrono::system_clock::now();
}

// Directly control robot by passing a q_dot vector [yaw, x, y]
void autoMR::direct_control(Vector3f q_dot)
{
    float currentPhi = 0;
    uint8_t robot_command_array[12] = {0};

    if( updateCurrentFullRobotState() )
    {
        currentPhi = current_full_state.pose_and_id.pose_state.q.yaw;

        robot_command_array[0] = STANDARD_MODE;

        VectorXi motor_commands = CalculateSpeedCommands(robot_data.kinematics_data.H0_R, robot_data.robot_configuration, q_dot, currentPhi, 1);

        for(int i = 0; i < 8; i++)
        {
            robot_command_array[i + 1] = motor_commands(i);
        }

        robot_command_array[9] = target_full_state.bucket_state.extension;
        robot_command_array[10] = target_full_state.bucket_state.tilt;
        robot_command_array[11] = target_full_state.LED_state.program;

        SendRobotCommands(&robot_command_array[0], 1);
    }
    else {
        freezeRobot();
    }
}

// Set what it means for the robot to go into default state
void autoMR::setDefaultState(FullRobotState const &new_default_state)
{
    default_state = new_default_state;
}

// Get the field data loaded by aruco
void autoMR::setFieldData()
{
    try {
        robot_data.robot_field_data = getFieldData();
    }
    catch(int& err) {
        std::cerr << "Failed to get field data" << std::endl;
        throw err;
    }
}

void autoMR::resumeOperation()
{
    stopRobot = false;
}

// Force the robot into default state
void autoMR::resetRobot()
{
    pushNewRobotState(default_state);
    stopRobot = false;
}

// Immediatelly stop the robot from moving
void autoMR::PANIC_STOP()
{
    stopRobot = true;
}

// Proportional controller, returns a speed scalar[0-1] to be used
float autoMR::P_Controller(Vector3f const &pose_error)
{
    Vector3f relative_size_of_error;

    relative_size_of_error(0,0) = abs( pose_error(0,0) ) / robot_data.robot_constraints.Error_Limit_Max_Yaw;
    relative_size_of_error(1,0) = abs( pose_error(1,0) ) / robot_data.robot_constraints.Error_Limit_Max_X;
    relative_size_of_error(2,0) = abs( pose_error(2,0) ) / robot_data.robot_constraints.Error_Limit_Max_Y;

    return findMaxAbsValue(relative_size_of_error);
}

// Retrives the newest full robot state if available and saves it in current_full_state, skips if unavailable
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

// Timesteps pose information using last known speed and acceleration data
int autoMR::interpolateDeadReckoningAndUpdate()
{
    typedef std::chrono::milliseconds ms;

    // stop timer and calculate time passed
    auto timer_stop = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = timer_stop - interpolation_time_start;

    resetInterpolationTimer();

    // cast to microseconds (TODO sync with time unit used in speed derivation)
    ms millis = std::chrono::duration_cast<ms>(elapsed);
    double time_for_latest_pass = millis.count();
    std::cout << "The time used for interpolation: " << time_for_latest_pass << std::endl;

    if(time_for_latest_pass != 0)
    {
        // for now neglect change in speed, /1000 because time is in ms and speed is in cm/s
        current_full_state.pose_and_id.pose_state.q.x += (time_for_latest_pass * current_full_state.pose_and_id.pose_state.q_dot.x)/1000; 
        current_full_state.pose_and_id.pose_state.q.y += (time_for_latest_pass * current_full_state.pose_and_id.pose_state.q_dot.y)/1000; 
        current_full_state.pose_and_id.pose_state.q.yaw += (time_for_latest_pass * current_full_state.pose_and_id.pose_state.q_dot.yaw)/1000;

        return 1;
    }
    else return 0;   
}

void autoMR::getBatteryStatus()
{
    float battery_status = 1;
    robot_data.battery_percentage = battery_status;
}

void autoMR::testSingleAMRHardware()
{
    for(int i = 0; i < 6; i++)
    {
        this->setCustomDirection(i, 220, 2000);
    }

    msDelay(1000);

    RGBColor my_color;
    my_color.r = 50;
    my_color.g = 150;
    my_color.b = 255;

    
    this->setCustomColor(15, my_color, BLINK_ONCE, 10);
    msDelay(3000);

    RobotConfiguration current_config = this->getRobotConfig();

    for(int i = 0; i < 10; i++)
    {
        this->setCustomBucket(current_config.Min_Bucket_Tilt, current_config.Min_Bucket_Extend);
        msDelay(1000);
        this->setCustomBucket(current_config.Max_Bucket_Tilt, current_config.Max_Bucket_Extend);
        msDelay(1000);
    }
}