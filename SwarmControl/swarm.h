#ifndef SWARM_H
#define SWARM_H

#include <map>
#include <vector>
#include <deque>

#include "RobotControl/autoMR.h"
#include "MotionPlanning/navigation.h"

/* **
* 0 = no metrics
* 1 = basic metrics
* 2 = all metrics */
#define ROBOT_METRICS_LVL 1

// holds swarm data(ids, battery lvls...)
struct SwarmData {
    
    int number_of_robots = 0;

    std::unordered_map<int, float> battery_statuses;
    std::unordered_map<int, RobotData> robots_data;
    std::vector<int> robot_ids;
};

struct SingleSetpoint {

    SinglePose pose = {0, 0, 0};
    SignalLED LED_state;
    BucketState bucket_state;
};

typedef std::unordered_map< int, std::vector<SingleSetpoint> > AllSetpointSets;
typedef std::unordered_map< int, FullStateTrajectory> AllFullStateTrajectories;

struct SwarmBox {

    // holds variables for velocity and acceleration averaging, memory size determined by ACCEL_MEMORY_SIZE
    std::unordered_map<int, std::deque<FullPoseState> > q_memory_map;
    std::unordered_map<int, std::chrono::_V2::system_clock::time_point > q_timer_start_map;

    // holds all FullStates accessed by ID
    std::unordered_map<int, FullRobotState> ID_FullState_Map;

    // holds all refferences to robot objects, accessed by ID 
    std::unordered_map<int, std::shared_ptr<autoMR> > ID_Unit_Map;
    std::vector<int> ids;
};

class Swarm {

    friend class autoMR;

    public:
        // holds all active AMRs
        SwarmBox swarm_box;
        
        // holds all swarm data like ids, battery lvls...
        SwarmData swarm_data;

        // queue for all new setpoints
        std::shared_ptr< rigtorp::SPSCQueue<AllFullStateTrajectories> > all_setpoints;

        Swarm(std::vector<int> idsVector)
        {
            // queue holding all of the robot desired states from the new pass
            all_setpoints = std::make_shared< rigtorp::SPSCQueue<AllFullStateTrajectories> >(2);
            
            // initialize robots
            initializeSwarm(idsVector);
        }

        ~Swarm(){}

        void addRobotToSwarm(int id);

        void removeRobotFromSwarm(int id);

        bool UpdateSwarm();

        void PANIC_STOP_SWARM();

        void getSwarmData();

        void testSwarmHardware();
    
    private:

        bool idExists(int id);

        int getAllNewSetpoints(AllFullStateTrajectories& all_new_setpoints);

        int generateTrajectory(FullStateTrajectory& output_trajectory, FullStateTrajectory const &input_states, std::string planner);

        int generateAndPushAllTrajectories();

        void initializeSwarm(std::vector<int> entered_ids);

        void startClock(int id);

        double getTimePassed(int id);

        SinglePose getVelocity(int id, SinglePose new_q, double timekeeper);

        SinglePose getAcceleration(int id, SinglePose, double timekeeper);
};

#endif // SWARM_H