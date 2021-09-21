// include 3rd party libs
#include "../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"
#include "../3rdParty/rapidjson/filereadstream.h"
#include "../3rdParty/rapidjson/rapidjson.h"
#include "../3rdParty/rapidjson/document.h"
#include "../3rdParty/rapidjson/istreamwrapper.h"

#include <Eigen/Dense>

#include <vector>
#include <fstream>

// Error codes
#define FATAL_ERROR 1
#define CASE_ERROR 2

struct SinglePose
{
    float yaw = 0;
    float x = 0;
    float y = 0;
};

struct FullPoseState
{
    SinglePose q;
    SinglePose q_dot;
    SinglePose q_dot_dot;
};

struct ChassisFullState
{
    int id;
    FullPoseState pose_state;
};

struct AllPoseStates
{
    std::vector<ChassisFullState> poses;
};

int getAllPoseStates(AllPoseStates& pose_holder);

void start_aruco_detection();