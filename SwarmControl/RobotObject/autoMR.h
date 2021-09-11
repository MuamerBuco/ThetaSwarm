#ifndef autoMR_H
#define autoMR_H

#include "RobotCommands/sendRobotCommands.h"
#include "TrajectoryPlanning/Kinematics/kinematics.h"
#include "../../ArucoTracking/arucoDetection.h"
#include "../../Common/common.h"

#include <thread>

// struct holding PD controller coefficients
struct PD_Controller_Coefficients {

    int Kp_yaw = 1;;
    int Kp_x = 1;
    int Kp_y = 1;
};

// struct holding robot data required for kinematics calculations
struct RobotKinematicsData {

    PD_Controller_Coefficients pd_coefficients;
    // TODO check if this can be not initialized to (4,3) and still work
    Eigen::MatrixXf H0_R;
};

// struct holding data about the robot(ID, battery, kinematics data, configuration data, client data )
struct RobotData {
    
    int ID;

    std::string robotIP = "192.168.1.999";
    int robotPort = 0;

    RobotConstraints robot_constraints;
    RobotConfiguration robot_configuration;
    RobotKinematicsData kinematics_data;

    float battery_percentage;
};

// struct holding bucket state(tilt, extension)
struct BucketState {
    float tilt = 0;
    float extension = 0;
};

// struct holding LED ring state(running program)
struct SignalLED {
    std::string program = "none";
};

// struct holding the current robot state(pose, LED and bucket)
struct FullRobotState {
    ChassisFullState current_pose_and_id;
    ChassisFullState target_pose_and_id;

    SignalLED LED_state;
    BucketState bucket_state;
};

// TODO change private and public
/* class defining the robot object
*   starts a thread that does the kinematics, path planning and command sending
*   the object data gets modified by invoking appropriate class methods from swarm control
*   the data is placed in robot state queue and consumed by the thread on per robot basis */ 
class autoMR
{
    public:

        FullRobotState current_full_state;
        FullRobotState default_state;
        FullRobotState stop_state;

        RobotData robot_data;

        autoMR() = default;

        autoMR(int id)
        {
            initializeRobot(id);

            worker_thread = std::thread(&autoMR::robot_control, this);
        }

        // add try catch because  join can throw so calling it without a try..catch from a destructor is reckless
        ~autoMR() { worker_thread.join(); };

    public:

        std::thread worker_thread;

        bool initializeRobot(int id);

        void loadConfig(std::string filePath, int id);

        void initializeRobotStates(int id);

        bool ReachedTarget();

        void updateCurrentFullRobotState();

        Eigen::Vector3f PD_Controller(Eigen::Vector3f pose_error);

        const ChassisFullState getLastRobotPose();

        const SignalLED getLastLEDState();

        const BucketState getLastBucketState();

        bool pushStateToThread(FullRobotState new_full_robot_state);
        
        void setDefaultState(FullRobotState new_default_state);

        void getBatteryStatus();
        
        void resetRobot();
        
        void PANIC_STOP();

        void launchWorkerThread();

        void robot_control();
};

#endif // autoMR_H