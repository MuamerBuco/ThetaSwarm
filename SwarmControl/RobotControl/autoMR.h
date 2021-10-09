#ifndef autoMR_H
#define autoMR_H

#include "sendRobotCommands.h"
#include "../MotionPlanning/kinematics.h"
#include "../../ArucoTracking/arucoDetection.h"

#include <thread>
#include <stdexcept>

enum command_modes {
    STANDARD_MODE = 1,
    CUSTOM_MOVE,
    CUSTOM_LED,
    CUSTOM_BUCKET
};

// struct holding PD controller coefficients
struct PD_Controller_Coefficients {

    int Kp_yaw = 1;
    int Kp_x = 1;
    int Kp_y = 1;
};

// struct holding robot data required for kinematics calculations
struct RobotKinematicsData {

    PD_Controller_Coefficients pd_coefficients;
    Eigen::MatrixXf H0_R;
};

struct RGBColor {

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

// struct holding data about the robot(ID, battery, kinematics data, configuration data, client data )
struct RobotData {
    
    int ID;

    std::string robotIP = "192.168.1.999";
    int robotPort = 0;

    RGBColor default_color;

    RobotConstraints robot_constraints;
    RobotConfiguration robot_configuration;
    RobotKinematicsData kinematics_data;

    float battery_percentage;
};

/* class defining the robot object
*   starts a thread that does the kinematics, path planning and command sending
*   the object data gets modified by invoking appropriate class methods from swarm control
*   the data is placed in robot state queue and consumed by the thread on per robot basis */ 
class autoMR
{
    public:
        FullRobotState current_full_state;
        FullRobotState target_full_state;

        FullRobotState default_state;
        FullRobotState stop_state;

        FullStateTrajectory full_state_trajectory;

        std::shared_ptr< rigtorp::SPSCQueue<FullRobotState> > LatestRobotState;
        std::shared_ptr< rigtorp::SPSCQueue<FullStateTrajectory> > TrajectorySet;
        std::shared_ptr< udp_client_server::udp_client > robot_client;

        std::atomic<bool> stopRobot = false;

        RobotData robot_data;

        autoMR() = default;

        autoMR(int id)
        {
            // queues the latest robot data(current pose, target pose, bucket state...) and trajectory set
            LatestRobotState = std::make_shared< rigtorp::SPSCQueue<FullRobotState> >(2);
            TrajectorySet = std::make_shared< rigtorp::SPSCQueue<FullStateTrajectory> >(2);

            // TODO1 for all try catch derive type from <stdexcept>
            try {
    
                initializeRobot(id);
            }
            catch( int& err ) {
                if(err == FATAL_ERROR){
                    std::cerr << "Failed to access configuration, aborting..." << std::endl;
                    std::abort();
                }
                else if(err == CASE_ERROR) {
                    std::cerr << "Failed to initialize robot ID: " << id << std::endl;
                    std::cerr << "Skipping..." << std::endl;
                    std::exit( EXIT_FAILURE );
                }
            }

            worker_thread = std::thread(&autoMR::robot_control, this);
        }

        ~autoMR() 
        { 
            try {
                worker_thread.join(); 
            } 
            catch(const std::system_error& e) {
                std::cerr << "Failed to join autoMR worker thread" << std::endl;
                std::exit( EXIT_FAILURE );
            }
            
        };

        void getBatteryStatus();
        
        void resetRobot();

        void resumeOperation();

        void PANIC_STOP();

        int setTrajectory(FullStateTrajectory const &full_trajectory);

        bool pushNewRobotState(FullRobotState const &new_full_robot_state);

        void setCustomDirection(uint8_t direction, uint8_t speed, int ms_delay);

        void setCustomColor(uint8_t index, RGBColor my_color, CustomLEDprograms mode, uint8_t ms_delay);

        void setCustomBucket(uint8_t tilt, uint8_t extend);

        void selfIdentify();

    
    private:

        std::thread worker_thread;

        bool initializeRobot(int id);

        bool loadConfig(std::string filePath, int id);

        void initializeRobotStates(int id);

        bool ReachedTarget();

        int updateCurrentFullRobotState();

        int setNextTrajectoryPoint();

        int updateFullTrajectory();

        Vector3f getPoseError();

        Eigen::Vector3f PD_Controller(Eigen::Vector3f const &pose_error);

        int getNextTrajectory();

        const ChassisFullState getLastRobotPose();

        const SignalLED getLastLEDState();

        const BucketState getLastBucketState();
        
        void setDefaultState(FullRobotState const &new_default_state);

        void freezeRobot(std::shared_ptr<udp_client_server::udp_client> client_object);

        void launchWorkerThread();

        void robot_control();
};

#endif // autoMR_H