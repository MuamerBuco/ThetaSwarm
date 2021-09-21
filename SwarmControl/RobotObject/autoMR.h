#ifndef autoMR_H
#define autoMR_H

#include "RobotCommands/sendRobotCommands.h"
#include "TrajectoryPlanning/Kinematics/kinematics.h"
#include "../../ArucoTracking/arucoDetection.h"
#include "../../util/util.h"

#include <thread>
#include <stdexcept>

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

enum LEDProgram {
    
    NONE,
    BLINK_RED,
    SOLID_RED,
    SOLID_BLUE,
    SOLID_GREEN,
};

// struct holding LED ring state(running program)
struct SignalLED {

    LEDProgram program = SOLID_BLUE;
};

struct SingleStateTrajectory {

    SinglePose pose = {0, 0, 0};
    SignalLED LED_state;
    BucketState bucket_state;
};

typedef std::vector<SingleStateTrajectory> FullStateTrajectory;

// struct holding the current robot state(pose, LED and bucket)
struct FullRobotState {
    
    ChassisFullState current_pose_and_id;
    ChassisFullState target_pose_and_id;

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

        FullStateTrajectory full_state_trajectory;

        std::shared_ptr< rigtorp::SPSCQueue<FullRobotState> > LatestRobotState;
        std::shared_ptr< rigtorp::SPSCQueue<FullStateTrajectory> > TrajectorySet;

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

        int setTrajectory(FullStateTrajectory* full_trajectory);

        bool pushNewRobotState(FullRobotState* new_full_robot_state);

    
    private:

        std::thread worker_thread;

        bool initializeRobot(int id);

        void loadConfig(std::string filePath, int id);

        void initializeRobotStates(int id);

        bool ReachedTarget();

        int updateCurrentFullRobotState();

        int setNextTrajectoryPoint();

        int updateFullTrajectory();

        Eigen::Vector3f PD_Controller(Eigen::Vector3f pose_error);

        int getNextTrajectory();

        const ChassisFullState getLastRobotPose();

        const SignalLED getLastLEDState();

        const BucketState getLastBucketState();
        
        void setDefaultState(FullRobotState new_default_state);

        void freezeRobot(std::shared_ptr<udp_client_server::udp_client> client_object);

        void launchWorkerThread();

        void robot_control();
};

#endif // autoMR_H