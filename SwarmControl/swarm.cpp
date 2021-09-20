#include "swarm.h"

#define ACCEL_MEMORY_SIZE 2

// TODO1 write speed, acceleration tests, derive practical limits

// convenience variable
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
    unsigned int counter = 0;
    for (auto const& x : swarm_box.ID_Unit_Map)
    {
        // access ID_FullState_Map by ID and get robot data and IDs
        swarm_data.robots_data.push_back(x.second->robot_data);
        swarm_data.robot_ids.push_back(x.first);

        // get battery statuses
        x.second->getBatteryStatus();
        swarm_data.battery_statuses.push_back(x.second->robot_data.battery_percentage);
        counter++;
    }

    swarm_data.number_of_robots = counter;
}

// creates n dynamic robot objects, need to be destructed with deinitializeSwarm()
void Swarm::initializeSwarm(std::vector<int> ids)
{
    using namespace std;
    
    FullRobotState initial_state;

    // create vel/acc queue and timer queue with size of ACCEL_MEMORY_SIZE
    std::deque<FullPoseState> initial_vel_acc_queue(ACCEL_MEMORY_SIZE, temp_state);
    std::deque<double> initial_time_memories_map(ACCEL_MEMORY_SIZE, 0.1);
    std::chrono::_V2::system_clock::time_point initial_timer_start;

    std::cout << "created variables in initialize swarm" << std::endl;

    // initialize robots, fill unit map with < ID, autoMR* > pairs, initialize timer, timkeeping and vel/acc maps, and initialize ID-State map
    for(int id : ids)
    {
        // create robot object and initialize maps
        autoMR * temp_amr = new autoMR(id);
        std::cout << "created new robot object in initialize swarm" << std::endl;
        swarm_box.ID_Unit_Map.insert( make_pair(id, temp_amr) );
        swarm_box.ID_FullState_Map.insert( make_pair(id, initial_state) );
        
        // initialize metric related data
        swarm_box.q_memory_map.insert( make_pair(id, initial_vel_acc_queue) );
        swarm_box.time_memories_map.insert( make_pair(id, initial_time_memories_map) );
        swarm_box.timer_start_map.insert( make_pair(id, initial_timer_start) );
        std::cout << "pushed shit into maps in initialize swarm" << std::endl;
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

    delete all_setpoints;
}

// overload + operator for SinglePoseVector custom struct
SinglePose operator+(SinglePose a, SinglePose b) 
{
    SinglePose S;

    S.yaw = a.yaw + b.yaw;
    S.x = a.x + b.x;
    S.y = a.y + b.y;

    return S;
}

// overload - operator for SinglePose custom struct
SinglePose operator-(SinglePose a, SinglePose b) 
{
    SinglePose S;

    S.yaw = a.yaw - b.yaw;
    S.x = a.x - b.x;
    S.y = a.y - b.y;

    return S;
}

// overload / operator for SinglePose custom struct
SinglePose operator/(SinglePose a, int b) 
{
    SinglePose S;
    // TODO1 after size of memory is established, byte shift this
    S.yaw = a.yaw / b;
    S.x = a.x / b;
    S.y = a.y / b;

    return S;
}

// take new q vector, find point by point distances using memory, find average value
// basically finds the displacement in the last ACCEL_MEMORY_SIZE  number of passes
SinglePose Swarm::getVelocity(int id, SinglePose new_q)
{
    SinglePose temp;
    // get the current deque holding Q state memory of robot[ID] 
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    temp = new_q - state_holder.at(0).q;

    // std::cout << "The new value for x: " << new_q.x << std::endl;

    // std::cout << "The old value for x: " << state_holder.at(0).q.x << std::endl;

    std::cout << "The new difference between last entry in q and new value for x: " << temp.x << std::endl;

    // average previous [ACCEL_MEMORY_SIZE] number of position differences
    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 1; i++ )
    {
        temp = temp + (state_holder.at(i+1).q - state_holder.at(i).q);
    }

    // std::cout << "The new summed value for x_dot: " << temp.x << std::endl;

    std::cout << "The new averaged value for x_dot: " << temp.x/ACCEL_MEMORY_SIZE << std::endl;


    // reduce by one, one datapoint of velocity is difference between 2 datapoints of position, 
    // so total number of generated datapoints for averaging is 1 lower than memory size
    return (new_q / (ACCEL_MEMORY_SIZE) );
}

// take new q_dot vector, 
SinglePose Swarm::getAcceleration(int id, SinglePose new_q_dot)
{
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 1; i++ )
    {
        new_q_dot = new_q_dot + (state_holder.at(i).q_dot - state_holder.at(i+1).q_dot);
    }

    new_q_dot = new_q_dot / (ACCEL_MEMORY_SIZE);

    return new_q_dot;
}

// starts the timer
void Swarm::startClock(int id)
{
     swarm_box.timer_start_map.find(id)->second = std::chrono::system_clock::now();
}

// stop clock when called, average new time together with memory, place new time in memory, return averaged time for one pass in miliseconds
double Swarm::getTimePassed(int id)
{
    typedef std::chrono::milliseconds ms;
    
    // stop timer
    auto timer_stop = std::chrono::system_clock::now();

    // calculate duration between clock starting for [ID] robot and clock stopping
    std::chrono::duration<double> elapsed = timer_stop - swarm_box.timer_start_map.find(id)->second;

    // cast to milliseconds
    ms milis = std::chrono::duration_cast<ms>(elapsed);
    double time_for_latest_pass = milis.count();

    // std::cout << "time before averaging passed in milliseconds: " << time_for_latest_pass << std::endl;

    // average latest time that passed with the previous
    for ( int i = 0; i < ACCEL_MEMORY_SIZE - 1; i++ )
    {
        time_for_latest_pass += (swarm_box.time_memories_map.find(id)->second.at(i));
    }

    // remove oldest entry in time table(ms)
    swarm_box.time_memories_map.find(id)->second.pop_back();

    // TODO1 byteShift
    time_for_latest_pass = time_for_latest_pass / (ACCEL_MEMORY_SIZE);

    // std::cout << "time after averaging passed in milliseconds: " << time_for_latest_pass << std::endl;

    // if the time exceeds 2 seconds, set it to 100ms to avoid averaging overflow
    if(time_for_latest_pass > 2000) time_for_latest_pass = 100;

    // add newest averaged time to time records
    swarm_box.time_memories_map.find(id)->second.push_front(time_for_latest_pass);

    // start the clock again
    startClock(id);

    // return the time it took in milliseconds
    return time_for_latest_pass;
}

// non blocking get for setpoints
int Swarm::getAllNewSetpoints(AllFullStateTrajectories& all_new_setpoints)
{
    // std::cout << "im in getAllNewSetpoints" << std::endl;
    if( all_setpoints->front() )
    {
        all_new_setpoints = *( all_setpoints->front() );
        all_setpoints->pop();
        return 1;
    }
    else {
        
        return 0;
    }
}

// get all new setpoints, generate trajectories and push to robots trajectory holder
// returns false if no new setpoints, writes to console if cant generate new trajectory for ID
int Swarm::generateAndPushAllTrajectories()
{
    AllFullStateTrajectories new_trajectories_map;
    if( getAllNewSetpoints(new_trajectories_map) )
    {
        for(auto single_trajectory : new_trajectories_map)
        {
            FullStateTrajectory new_trajectory;
            if( generateTrajectory(new_trajectory, single_trajectory.second, "BITstar") )
            {
                // push new trajectory to robot
                swarm_box.ID_Unit_Map.find(single_trajectory.first)->second->setTrajectory(new_trajectory);
            }
            else {
                std::cout << "Couldn't generate trajectory for ID: " << single_trajectory.first << std::endl;
            }
        }
        return 1;
    }
    else {
        // std::cout << "No new setpoints recognized" << std::endl;
        return 0;
    }
}

// Generate trajectory for inputs_states set of setpoints
int Swarm::generateTrajectory(FullStateTrajectory& output_trajectory, FullStateTrajectory input_states, std::string planner)
{
    // if trajectory created succesfuly, write it to output trajectory
    FullStateTrajectory temp_trajectory;
    if( makeTrajectory( temp_trajectory, input_states, planner) )
    {
        output_trajectory = temp_trajectory;
        return 1;
    }
    else {
        // std::cout << "Couldn't generate trajectory" << std::endl;
        return 0;
    }
}

// return true if id is in the swarm
bool Swarm::idExists(int id)
{
    for(int test_id : swarm_box.ids)
    {
        if(test_id == id) return 1;
    }

    return 0;
}

// update swarm
bool Swarm::UpdateSwarm()
{
    AllPoseStates allPoses;
    FullRobotState FullStateHolder;

    if( getAllPoseStates(allPoses) )
    {
        double timekeeper;

        for (ChassisFullState pose : allPoses.poses)
        {
            int id = pose.id;

            if( idExists(id) )
            {
                timekeeper = getTimePassed(id);

                // get new q
                temp_state = pose.pose_state;

                // TODO1 rethink the velocity and acceleration calculations, maybe remove averaging 

                // calculate new speed based on new position (cm/s)
                FullStateHolder.current_pose_and_id = pose;
                FullStateHolder.current_pose_and_id.pose_state.q_dot = (getVelocity(id, temp_state.q) / timekeeper) / 100;
                temp_state.q_dot = FullStateHolder.current_pose_and_id.pose_state.q_dot;
                // std::cout << "got the velocity: " << FullStateHolder.current_pose_and_id.pose_state.q_dot.x << std::endl;

                // std::cout << "The timekeeper(in ms): " << timekeeper << std::endl;

                // calculate new acceleration based on new speed (cm/s2)
                FullStateHolder.current_pose_and_id.pose_state.q_dot_dot = (getAcceleration(id, temp_state.q_dot) / timekeeper) / 100;
                temp_state.q_dot_dot = FullStateHolder.current_pose_and_id.pose_state.q_dot_dot;
                // std::cout << "got the accel: " << FullStateHolder.current_pose_and_id.pose_state.q_dot_dot.x << std::endl;

                // remove oldest entry in memory
                swarm_box.q_memory_map.find(id)->second.pop_back();

                // push temp_state to q memory, holding new q, q_dot and q_dot_dot
                swarm_box.q_memory_map.find(id)->second.push_front(temp_state);

                // place the proper (full state / ID) pair in ID-State map
                swarm_box.ID_FullState_Map.find(id)->second = FullStateHolder;

                // push new state to the robot
                bool temp = swarm_box.ID_Unit_Map.find(id)->second->pushNewRobotState( swarm_box.ID_FullState_Map.find(id)->second );
            }

            // generate and push all trajectories if any new setpoints are found, non blocking
            if( generateAndPushAllTrajectories() )
            {
                std::cout << "Pushed new trajectories" << std::endl;
            }
        }
        return 1;
    }
    
    return 0;
}


