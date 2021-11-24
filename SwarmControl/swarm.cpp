#include "swarm.h"

#define ACCEL_MEMORY_SIZE 2

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
    unsigned int number_of_robots = 0;

    for (auto const& x : swarm_box.ID_Unit_Map)
    {
        // access ID_FullState_Map by ID and get robot data and IDs
        swarm_data.robots_data.insert( std::make_pair(x.first ,x.second->robot_data) );
        swarm_data.robot_ids.push_back(x.first);

        // get battery statuses
        x.second->getBatteryStatus();
        swarm_data.battery_statuses.insert( std::make_pair(x.first, x.second->robot_data.battery_percentage) );
        number_of_robots++;
    }

    swarm_data.number_of_robots = number_of_robots;
}

// create robot objects and initialize all robot related data
void Swarm::initializeSwarm(std::vector<int> entered_ids)
{
    using namespace std;
    
    FullRobotState initial_state;
    FullPoseState temp_state;

    // create vel/acc queue and timer queue with size of ACCEL_MEMORY_SIZE
    std::deque<FullPoseState> initial_vel_acc_queue(ACCEL_MEMORY_SIZE, temp_state);
    std::chrono::_V2::system_clock::time_point initial_timer_start;

    // initialize robots, fill unit map with < ID, autoMR* > pairs, initialize timer, timkeeping and vel/acc maps, and initialize ID-State map
    for(int id : entered_ids)
    {
        // create robot object and initialize maps
        try {
            auto temp_amr = std::make_shared< autoMR >(id);

            swarm_box.ID_Unit_Map.insert( make_pair(id, temp_amr) );
            swarm_box.ID_FullState_Map.insert( make_pair(id, initial_state) );
            
            // initialize metric related data
            swarm_box.q_memory_map.insert( make_pair(id, initial_vel_acc_queue) );
            swarm_box.q_timer_start_map.insert( make_pair(id, initial_timer_start) );

            swarm_box.ids.push_back(id);          
        }
        catch(const int& err) {
            std::cerr << "Swarm failed to initialize Robot ID: " << id << std::endl;
            std::cerr << "Skipping..." << std::endl;
            //std::exit( EXIT_FAILURE );
        }
        catch(const std::bad_alloc& e) {
            std::cerr << "Allocation failed: " << e.what() << std::endl;
            //std::exit( EXIT_FAILURE );
        }
    }

    getSwarmData();
}

void Swarm::addRobotToSwarm(int id)
{
    FullRobotState initial_state;
    FullPoseState temp_state;

    // create vel/acc queue and timer queue with size of ACCEL_MEMORY_SIZE
    std::deque<FullPoseState> initial_vel_acc_queue(ACCEL_MEMORY_SIZE, temp_state);
    std::chrono::_V2::system_clock::time_point initial_timer_start;

    // create robot object and initialize maps
    auto temp_amr = std::make_shared< autoMR >(id); 
    swarm_box.ID_Unit_Map.insert( std::make_pair(id, temp_amr) );
    swarm_box.ID_FullState_Map.insert( std::make_pair(id, initial_state) );
    
    // initialize metric related data
    swarm_box.q_memory_map.insert( std::make_pair(id, initial_vel_acc_queue) );
    swarm_box.q_timer_start_map.insert( std::make_pair(id, initial_timer_start) );

    // access ID_FullState_Map by ID and get robot data and IDs
    swarm_data.robots_data.insert( std::make_pair(id ,swarm_box.ID_Unit_Map.find(id)->second->robot_data) );
    swarm_data.robot_ids.push_back(id);

    // get battery statuses
    swarm_box.ID_Unit_Map.find(id)->second->getBatteryStatus();
    swarm_data.battery_statuses.insert( std::make_pair(id, swarm_box.ID_Unit_Map.find(id)->second->robot_data.battery_percentage) );

    swarm_data.number_of_robots += 1;
}

void Swarm::removeRobotFromSwarm(int id)
{
    swarm_box.ID_FullState_Map.erase(id);
    swarm_box.ID_Unit_Map.erase(id);

    swarm_box.q_memory_map.erase(id);
    swarm_box.q_timer_start_map.erase(id);

    swarm_box.ids.erase(std::remove(swarm_box.ids.begin(), swarm_box.ids.end(), id), swarm_box.ids.end());

    swarm_data.battery_statuses.erase(id);
    swarm_data.robots_data.erase(id);

    swarm_data.robot_ids.erase(std::remove(swarm_data.robot_ids.begin(), swarm_data.robot_ids.end(), id), swarm_data.robot_ids.end());

    swarm_data.number_of_robots -= 1;
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
SinglePose operator/(SinglePose a, float b) 
{
    SinglePose S;
    // TODO1 after size of memory is established, byte shift this
    S.yaw = a.yaw / b;
    S.x = a.x / b;
    S.y = a.y / b;

    return S;
}

// overload / operator for SinglePose custom struct
SinglePose operator*(SinglePose a, float b) 
{
    SinglePose S;
    // TODO1 after size of memory is established, byte shift this
    S.yaw = a.yaw * b;
    S.x = a.x * b;
    S.y = a.y * b;

    return S;
}

// take new q vector, find point by point distances using memory, find average value
// basically finds the displacement in the last ACCEL_MEMORY_SIZE  number of passes
SinglePose Swarm::getVelocity(int id, SinglePose new_q, double time_passed)
{
    // get the current deque holding Q state memory of robot[ID] 
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    new_q = new_q - state_holder.at(0).q;

    std::cout << "The new q difference -x: " << new_q.x << std::endl;
    std::cout << "The new q difference -y: " << new_q.y << std::endl;
    std::cout << "The new q difference -yaw: " << new_q.yaw << std::endl;

    std::cout << "The new time passed in velocity: " << time_passed << std::endl;

    SinglePose cm_per_sec = (new_q / time_passed) * 1000;

    // reduce by one, one datapoint of velocity is difference between 2 datapoints of position, 
    // so total number of generated datapoints for averaging is 1 lower than memory size
    return cm_per_sec;
}

// take new q_dot vector, 
SinglePose Swarm::getAcceleration(int id, SinglePose new_q_dot, double time_passed)
{
    // get the current deque holding Q state memory of robot[ID] 
    std::deque<FullPoseState> state_holder = swarm_box.q_memory_map.find(id)->second;

    new_q_dot = new_q_dot - state_holder.at(0).q_dot;

    std::cout << "The new q_dot difference -x: " << new_q_dot.x << std::endl;
    std::cout << "The new q_dot difference -y: " << new_q_dot.y << std::endl;
    std::cout << "The new q_dot difference -yaw: " << new_q_dot.yaw << std::endl;

    std::cout << "The new time passed in velocity: " << time_passed << std::endl;

    SinglePose cm_per_s_per_s = (new_q_dot / time_passed) * 1000;

    // reduce by one, one datapoint of velocity is difference between 2 datapoints of position, 
    // so total number of generated datapoints for averaging is 1 lower than memory size
    return cm_per_s_per_s;
}

// starts the timer
void Swarm::startClock(int id)
{
    swarm_box.q_timer_start_map.find(id)->second = std::chrono::system_clock::now();
}

// stop clock when called, average new time together with memory, place new time in memory, return averaged time for one pass in miliseconds
double Swarm::getTimePassed(int id)
{
    typedef std::chrono::milliseconds ms;
    
    // stop timer
    auto timer_stop = std::chrono::system_clock::now();

    // calculate duration between clock starting for [ID] robot and clock stopping
    std::chrono::duration<double> elapsed = timer_stop - swarm_box.q_timer_start_map.find(id)->second;

    // cast to milliseconds
    ms milis = std::chrono::duration_cast<ms>(elapsed);
    double time_for_latest_pass = milis.count();

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
        FullStateTrajectory new_trajectory;
        for(auto single_trajectory : new_trajectories_map)
        {
            if( generateTrajectory(new_trajectory, single_trajectory.second, "BITstar") )
            {
                // push new trajectory to robot
                swarm_box.ID_Unit_Map.find(single_trajectory.first)->second->setTrajectory(new_trajectory);
            }
            else {
                std::cerr << "Couldn't generate trajectory for ID: " << single_trajectory.first << std::endl;
            }
        }
        return 1;
    }
    else {
        // std::cerr << "No new setpoints recognized" << std::endl;
        return 0;
    }
}

// Generate trajectory for inputs_states set of setpoints
int Swarm::generateTrajectory(FullStateTrajectory& output_trajectory, FullStateTrajectory const  &input_states, std::string planner)
{
    // if trajectory created succesfuly, write it to output trajectory
    if( makeTrajectory( output_trajectory, input_states, planner) )
    {
        return 1;
    }
    else {
        std::cerr << "Couldn't generate trajectory" << std::endl;
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

/*
* For each active robot:
* Obtain new values of [yaw, x, y] from pose estimation
* Calculate velocity and acceleration
* Update swarm data with new values
* Push new full state to robots */
bool Swarm::UpdateSwarm()
{
    AllPoseStates allPoses;
    FullRobotState current_full_state;
    FullPoseState temp_state;

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

                // current_full_state.pose_and_id.pose_state.q_dot = (getVelocity(id, temp_state.q) / timekeeper) / 100;
                // temp_state.q_dot = current_full_state.pose_and_id.pose_state.q_dot;
                
                // calculate new speed based on new position (cm/s)
                temp_state.q_dot = getVelocity(id, temp_state.q, timekeeper);

                // std::cout << "The timekeeper(in ms): " << timekeeper << std::endl;

                // calculate new acceleration based on new speed (cm/s2)
                // current_full_state.pose_and_id.pose_state.q_dot_dot = (getAcceleration(id, temp_state.q_dot) / timekeeper) / 100;
                // temp_state.q_dot_dot = current_full_state.pose_and_id.pose_state.q_dot_dot;
                // std::cout << "got the accel: " << current_full_state.pose_and_id.pose_state.q_dot_dot.x << std::endl;

                temp_state.q_dot_dot = getAcceleration(id, temp_state.q_dot, timekeeper);

                // save new yaw,x,y and q values into the new full state holder
                current_full_state.pose_and_id = pose;
                current_full_state.pose_and_id.pose_state = temp_state;

                // push new pose state to q memory, holding new q, q_dot and q_dot_dot
                swarm_box.q_memory_map.find(id)->second.push_front(temp_state);

                // remove oldest entry in memory
                swarm_box.q_memory_map.find(id)->second.pop_back();

                // place the proper (full state / ID) pair in ID-State map
                swarm_box.ID_FullState_Map.find(id)->second = current_full_state;

                // push new state to the robot
                swarm_box.ID_Unit_Map.find(id)->second->pushNewRobotState( swarm_box.ID_FullState_Map.find(id)->second );

#if ROBOT_METRICS_LVL > 0
                std::clog << std::endl << "Metrics for robot ID: " << id << std::endl;
                std::clog << "Values for POSE: [X, Y, YAW]" << std::endl;
                std::clog << "[ " << current_full_state.pose_and_id.pose_state.q.x << ", " << current_full_state.pose_and_id.pose_state.q.y << ", " << current_full_state.pose_and_id.pose_state.q.yaw << " ]" << std::endl;
                std::clog << "Values for SPEED: [X_DOT, Y_DOT, YAW_DOT]" << std::endl;
                std::clog << "[ " << current_full_state.pose_and_id.pose_state.q_dot.x << ", " << current_full_state.pose_and_id.pose_state.q_dot.y << ", " << current_full_state.pose_and_id.pose_state.q_dot.yaw << " ]" << std::endl;
                std::clog << "Values for ACCELERATION: [X_DOT_DOT, Y_DOT_DOT, YAW_DOT_DOT]" << std::endl;
                std::clog << "[ " << current_full_state.pose_and_id.pose_state.q_dot_dot.x << ", " << current_full_state.pose_and_id.pose_state.q_dot_dot.y << ", " << current_full_state.pose_and_id.pose_state.q_dot_dot.yaw << " ]" << std::endl;
#endif
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

void Swarm::testSwarmHardware()
{
    for(auto robot : swarm_box.ID_Unit_Map)
    {
        robot.second->testSingleAMRHardware();
    }
}
