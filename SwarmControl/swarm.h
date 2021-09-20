#ifndef Swarm_H
#define Swarm_H

#include "RobotObject/autoMR.h"
#include "PathPlanning/navigation.h"

#include <map>
#include <vector>
#include <deque>

// struct holding swarm data(ids, battery lvls)
struct SwarmData {
    
    int number_of_robots = 0;

    std::vector<float> battery_statuses;
    std::vector<int> robot_ids;
    std::vector<RobotData> robots_data;
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
    std::map< int, std::deque<FullPoseState> > q_memory_map;
    std::map< int, std::deque<double> > time_memories_map;
    std::map< int, std::chrono::_V2::system_clock::time_point > timer_start_map;

    // AllFullStateTrajectories full_state_trajectories_map;

    std::map<int, FullRobotState> ID_FullState_Map;
    std::map<int, autoMR*> ID_Unit_Map;
    std::vector<int> ids;
};

class Swarm {

    friend class autoMR;

    public:
        // holds all active AMRs
        SwarmBox swarm_box;
        
        // holds all swarm data like ids, battery lvls...
        SwarmData swarm_data;

        // Queue for all new setpoints
        rigtorp::SPSCQueue<AllFullStateTrajectories>* all_setpoints;

        Swarm()
        {
            // queue holding all of the robot desired states from the new pass TODO1 maybe do smart pointer
            all_setpoints = new rigtorp::SPSCQueue<AllFullStateTrajectories>(2);
            
            // enter robot IDs
            swarm_box.ids = {10, 15};
            std::vector<int> idsVector = swarm_box.ids;

            std::cout << "Prepared IDs" << std::endl;

            // initialize robot objects, 
            initializeSwarm(idsVector);
        }

        ~Swarm()
        {
            deinitializeSwarm();
        }

        // TODO implement
        void addRobotToSwarm();

        void removeRobotFromSwarm();
        /////////////////////////////////////////
        bool UpdateSwarm();

        void PANIC_STOP_SWARM();

        void getSwarmData();
    
    private:
        /////// TODO switch to pointers

        bool idExists(int id);

        int getAllNewSetpoints(AllFullStateTrajectories& all_new_setpoints);

        int generateTrajectory(FullStateTrajectory& output_trajectory, FullStateTrajectory input_states, std::string planner);

        int generateAndPushAllTrajectories();

        void initializeSwarm(std::vector<int> ids);

        void deinitializeSwarm();

        void startClock(int id);

        double getTimePassed(int id);

        SinglePose getVelocity(int id, SinglePose new_q);

        SinglePose getAcceleration(int id, SinglePose new_q_dot);

        

};

#endif // Swarm_H