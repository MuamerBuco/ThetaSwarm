#ifndef autoMR_H
#define autoMR_H

#include "RobotCommands/sendMotorCommands.h"
#include "TrajectoryPlanning/Kinematics/kinematics.h"
#include "../../ArucoTracking/arucoDetection.h"
#include "../../Common/common.h"

#include <thread>

#define PROPORTIONAL_CONTROL_COEFFICIENT_P 1
#define PROPORTIONAL_CONTROL_COEFFICIENT_X 1
#define PROPORTIONAL_CONTROL_COEFFICIENT_Y 1

#define PRECISION_MARGIN_YAW 1
#define PRECISION_MARGIN_X 1
#define PRECISION_MARGIN_Y 1

// struct holding bucket state(tilt, extension)
struct BucketState {
    float tilt;
    float extension;
};

// struct holding LED ring state(running program)
struct SignalLED {
    std::string program;
};

// struct holding PD controller coefficients
struct PD_Controller_Coefficients
{
    int Kp_p;
    int Kp_x;
    int Kp_y;
};

// struct holding data about the robot(ID, battery)
struct RobotData {
    PD_Controller_Coefficients pd_coefficients;
    int ID;
    float battery_percentage;
};

// struct holding the current robot state(pose, LED and bucket)
struct FullRobotState {
    ChassisPoseAndID current_pose_and_id;
    ChassisPoseAndID target_pose_and_id;

    SignalLED LED_state;
    BucketState bucket_state;
};

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

        std::string robotIP;
        int robotPort;

        autoMR(int id)
        {
            initializeRobot(id);

            worker_thread = std::thread(&autoMR::robot_control, this);
        }

        ~autoMR() { worker_thread.join(); };

    private:

        std::thread worker_thread;

        bool initializeRobot(int id);

        void initializePDCoefficients();

        void initializeRobotStates(int id);

        bool ReachedTarget();

        FullRobotState getCurrentFullRobotState();

        // TODO make a per robot basis struct holding HOR data and top lvl configuration file, currently all is in kinematics.cpp
        //void initialize_H_0_R();

        Eigen::Vector3f PD_Controller(Eigen::Vector3f pose_error);

        const ChassisPoseAndID getLastRobotPose();

        const SignalLED getLastLEDState();

        const BucketState getLastBucketState();

        const bool pushStateToThread(FullRobotState new_full_robot_state);
        
        void setDefaultState(FullRobotState new_default_state);
        
        void resetRobot();
        
        void PANIC_STOP();

        void launchWorkerThread();

        void robot_control();
};

#endif // autoMR_H