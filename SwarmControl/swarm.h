#ifndef Swarm_H
#define Swarm_H

#include "RobotObject/autoMR.h"
#include "PathPlanning/navigation.h"

#include <map>
#include <vector>
#include <deque>

// holds swarm data(ids, battery lvls...)
struct SwarmData {
    
    int number_of_robots = 0;

    std::map<int, float> battery_statuses;
    std::map<int, RobotData> robots_data;
    std::vector<int> robot_ids;
};

struct SingleSetpoint {

    SinglePose pose = {0, 0, 0};
    SignalLED LED_state;
    BucketState bucket_state;
};

typedef std::map< int, std::vector<SingleSetpoint> > AllSetpointSets;
typedef std::map< int, FullStateTrajectory> AllFullStateTrajectories;

struct SwarmBox {

    // holding variables for velocity and acceleration averaging, memory size determined by ACCEL_MEMORY_SIZE
    std::map<int, std::deque<FullPoseState> > q_memory_map;
    std::map<int, std::deque<double> > time_memories_map;
    std::map<int, std::chrono::_V2::system_clock::time_point > timer_start_map;

    // holds all FullStates accessed by ID
    std::map<int, FullRobotState> ID_FullState_Map;

    // holds all pointers to robot objects, accessed by ID 
    std::map<int, std::shared_ptr<autoMR> > ID_Unit_Map;
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
            
            // enter robot IDs
            swarm_box.ids = idsVector;

            // initialize robots
            initializeSwarm();
        }

        ~Swarm(){}

        void addRobotToSwarm(int id);

        void removeRobotFromSwarm(int id);

        bool UpdateSwarm();

        void PANIC_STOP_SWARM();

        void getSwarmData();
    
    private:

        bool idExists(int id);

        int getAllNewSetpoints(AllFullStateTrajectories& all_new_setpoints);

        int generateTrajectory(FullStateTrajectory& output_trajectory, FullStateTrajectory input_states, std::string planner);

        int generateAndPushAllTrajectories();

        void initializeSwarm();

        void startClock(int id);

        double getTimePassed(int id);

        SinglePose getVelocity(int id, SinglePose new_q);

        SinglePose getAcceleration(int id, SinglePose new_q_dot);
};

#endif // Swarm_H