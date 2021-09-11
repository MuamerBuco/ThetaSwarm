#include "swarm.h"

using namespace std;

#define ACCEL_MEMORY_SIZE 4

// TODO write speed, acceleration tests, derive practical limits

// queue holding all of the robot desired states from the new pass 
// rigtorp::SPSCQueue<vector<FullRobotState> > allRobotStates(2);
AllPoseStates myPoses;
FullRobotState FullStateHolder;

// does nothing, only used to initialize
FullPoseState temp_state;

// stops the entire swarm
void Swarm::PANIC_STOP_SWARM()
{
    for (auto const& x : swarm_box.ID_Unit_Map)
    {
        // access ID_FullState_Map by ID and invoke panic stop
        x.second->PANIC_STOP();
    }
}

// fills in swarm data
void Swarm::getSwarmData()
{
    for (auto const& x : swarm_box.ID_Unit_Map)
    {
        // access ID_FullState_Map by ID and get robot data and IDs
        swarm_data.robots_data.push_back(x.second->robot_data);
        swarm_data.robot_ids.push_back(x.first);

        // get battery statuses
        x.second->getBatteryStatus();
        swarm_data.battery_statuses.push_back(x.second->robot_data.battery_percentage);
    }
}

// creates n dynamic robot objects, need to be destructed with deinitializeSwarm()
// TODO check out if it->second is the right choice
void Swarm::initializeSwarm(std::vector<int> ids)
{
    FullRobotState initial_state;

    // create vel/acc queue and timer queue with size of ACCEL_MEMORY_SIZE
    std::deque<FullPoseState> initial_vel_acc_queue(ACCEL_MEMORY_SIZE, temp_state);
    std::deque<float> initial_time_memories_map(ACCEL_MEMORY_SIZE, 0.1);
    std::chrono::_V2::system_clock::time_point initial_timer_start;

    // make robot objects, fill unit map with < ID, autoMR* > pairs, initialize timer, timkeeping and vel/acc maps, and initialize ID-State map
    for(int id : ids)
    {
        // create robot object and initialize maps
        autoMR * temp_amr = new autoMR(id);
        swarm_box.ID_Unit_Map.insert( make_pair(id, temp_amr) );
        swarm_box.ID_FullState_Map.insert( make_pair(id, initial_state) );
        
        // initialize metric related data
        swarm_box.q_memory_map.insert( make_pair(id, initial_vel_acc_queue) );
        swarm_box.time_memories_map.insert( make_pair(id, initial_time_memories_map) );
        swarm_box.timer_start_map.insert( make_pair(id, initial_timer_start) );
    }

    // initialize each robot
    for( auto const& x : swarm_box.ID_Unit_Map )
    {
        x.second->initializeRobot(x.first);
    }
}

// clean up
void Swarm::deinitializeSwarm()
{
    // delete all autoMR objects
    for( auto const& x : swarm_box.ID_Unit_Map )
    {
        delete x.second;
    }
}

// overload + operator for SinglePoseVector custom struct
SinglePoseVector operator+(SinglePoseVector a, SinglePoseVector b) 
{
    SinglePoseVector S;

    S.yaw = a.yaw + b.yaw;
    S.x = a.x + b.x;
    S.y = a.y + b.y;

    return S;
}

// overload - operator for SinglePoseVector custom struct
SinglePoseVector operator-(SinglePoseVector a, SinglePoseVector b) 
{
    SinglePoseVector S;

    S.yaw = a.yaw - b.yaw;
    S.x = a.x - b.x;
    S.y = a.y - b.y;

    return S;
}

// overload / operator for SinglePoseVector custom struct
SinglePoseVector operator/(SinglePoseVector a, int b) 
{
    SinglePoseVector S;
    // TODO after size of memory is established, byte shift this
    S.yaw = a.yaw / b;
    S.x = a.x / b;
    S.y = a.y / b;

    return S;
}

// take new q vector, find point by point distances using memory, find average value
// basically finds the displacement in the last ACCEL_MEMORY_SIZE  number of passes
SinglePoseVector Swarm::getVelocity(int id, SinglePoseVector new_q)
{
    // get the current deque holding Q state memory of robot[ID] 
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    // average previous [ACCEL_MEMORY_SIZE] number of position differences
    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 2; i++ )
    {
        new_q = new_q + (state_holder.at(i).q - state_holder.at(i+1).q);
    }

    // reduce by one, one datapoint of velocity is difference between 2 datapoints of position, 
    // so total number of generated datapoints for averaging is 1 lower than memory size
    return new_q / (ACCEL_MEMORY_SIZE - 1);
}

// take new q_dot vector, 
SinglePoseVector Swarm::getAcceleration(int id, SinglePoseVector new_q_dot)
{
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 2; i++ )
    {
        new_q_dot = new_q_dot + (state_holder.at(i).q_dot - state_holder.at(i+1).q_dot);
    }

    new_q_dot = new_q_dot / (ACCEL_MEMORY_SIZE - 1);

    return new_q_dot;
}

// starts the timer
void Swarm::startClock(int id)
{
     swarm_box.timer_start_map.find(id)->second = std::chrono::system_clock::now();
}

// stop clock when called, average new time together with memory, place new time in memory, return averaged time for one pass in seconds
float Swarm::getTimePassed(int id)
{
    typedef std::chrono::milliseconds ms;
    
    // stop timer
    auto timer_stop = std::chrono::system_clock::now();

    
    // calculate duration between clock starting for [ID] robot and clock stopping
    std::chrono::duration<float> elapsed = timer_stop - swarm_box.timer_start_map.find(id)->second;

    // cast to milliseconds
    ms milis = std::chrono::duration_cast<ms>(elapsed);
    float time_for_latest_pass = elapsed.count();

    // remove oldest entry in time table(ms)
    swarm_box.time_memories_map.find(id)->second.pop_back();

    // average latest time that passed with the previous
    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 2; i++ )
    {
        time_for_latest_pass += (swarm_box.time_memories_map.find(id)->second.at(i) - swarm_box.time_memories_map.find(id)->second.at(i+1));
    }

    // TODO byteShift
    time_for_latest_pass = time_for_latest_pass / (ACCEL_MEMORY_SIZE -1);

    // add newest averaged time to time records
    swarm_box.time_memories_map.find(id)->second.push_front(time_for_latest_pass);

    // start the clock again
    startClock(id);

    // return the time it took in seconds, even tho record is in [ms]
    return time_for_latest_pass * 1000;
}

// update swarm
bool Swarm::UpdateSwarm()
{
    myPoses = getAllPoseStates();

    float timekeeper;

    for (ChassisFullState pose : myPoses.poses)
    {
        int id = pose.id;
        timekeeper = getTimePassed(id);

        // get new q
        temp_state = pose.pose_state;

        // remove oldest entry in memory
        swarm_box.q_memory_map.find(id)->second.pop_back();

        // calculate new speed based on new position (cm/s)
        FullStateHolder.current_pose_and_id = pose;
        FullStateHolder.current_pose_and_id.pose_state.q_dot = getVelocity(id, temp_state.q) / timekeeper;
        temp_state.q_dot = FullStateHolder.current_pose_and_id.pose_state.q_dot;

        // calculate new acceleration based on new speed (cm/s2)
        FullStateHolder.current_pose_and_id.pose_state.q_dot_dot = getAcceleration(id, temp_state.q_dot) / timekeeper;
        temp_state.q_dot_dot = FullStateHolder.current_pose_and_id.pose_state.q_dot_dot;

        // push temp_state to q memory, holding new q, q_dot and q_dot_dot
        swarm_box.q_memory_map.find(id)->second.push_front(temp_state);

        // TODO 
        // implement getter functions from the trajectory 
        FullStateHolder.target_pose_and_id = getTargetPose(id);
        FullStateHolder.bucket_state = getTargetBucketState(id); // TODO for bucket send the id of an action, not the tilt
        FullStateHolder.LED_state = getTargetLEDState(id);

        // place the proper (full state / ID) pair in ID-State map
        swarm_box.ID_FullState_Map.find(id)->second = FullStateHolder;
        // ID_FullState_Map.insert( make_pair(id, FullStateHolder) );
    }

    for (auto const& x : swarm_box.ID_Unit_Map)
    {
        // access ID_FullState_Map by ID and pass the full state to the appropriate robot thread
        x.second->pushStateToThread( swarm_box.ID_FullState_Map.find(x.first)->second );
    }

    return 1;
}


