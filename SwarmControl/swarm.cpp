#include "swarm.h"
#include "RobotObject/autoMR.h"

#include <vector>

using namespace std;

// queue holding all of the robot desired states from the new pass 
rigtorp::SPSCQueue<std::vector<FullRobotState> > allRobotStates(2);

// struct holding swarm data(ids, battery lvls)
struct SwarmData {
    int number_of_robots;

    vector<float> battery_statuses;
    vector<int> robot_ids;
};

bool Swarm::UpdateSwarm()
{
    return 1;
}

void Swarm::PANIC_STOP_SWARM()
{

}

void Swarm::getSwarmData()
{

}



