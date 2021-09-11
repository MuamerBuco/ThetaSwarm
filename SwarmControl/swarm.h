#ifndef Swarm_H
#define Swarm_H

#include "RobotObject/autoMR.h"

#include <map>
#include <vector>
#include <deque>

// struct holding swarm data(ids, battery lvls)
// TODO somehow get number of robots
struct SwarmData {
    
    int number_of_robots = 0;

    std::vector<float> battery_statuses;
    std::vector<int> robot_ids;
    std::vector<RobotData> robots_data;
};

// struct SwarmFullState {
//     std::vector<FullRobotState> nextSwarmState;
// };

struct SwarmBox {

    // holding variables for velocity and acceleration averaging, memory size determined by ACCEL_MEMORY_SIZE
    std::map< int, std::deque<FullPoseState> > q_memory_map;
    std::map< int, std::deque<float> > time_memories_map;
    std::map< int, std::chrono::_V2::system_clock::time_point > timer_start_map;

    std::map<int, FullRobotState> ID_FullState_Map;
    std::map<int, autoMR*> ID_Unit_Map;
    std::vector<int> ids;
};

class Swarm {

    // will be using AMR class functions to build swarm control functions
    friend class autoMR;

    // vector for all active robot objects
    SwarmBox swarm_box;
    
    //SwarmFullState swarm_state;
    SwarmData swarm_data;

    Swarm()
    {
        // enter robot IDs
        swarm_box.ids = {10, 15};
        std::vector<int> idsVector = swarm_box.ids;

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
    ///////

    void initializeSwarm(std::vector<int> ids);

    void deinitializeSwarm();

    bool UpdateSwarm();

    void startClock(int id);

    float getTimePassed(int id);

    SinglePoseVector getVelocity(int id, SinglePoseVector new_q);

    SinglePoseVector getAcceleration(int id, SinglePoseVector new_q_dot);

    void PANIC_STOP_SWARM();

    void getSwarmData();

};

#endif // Swarm_H