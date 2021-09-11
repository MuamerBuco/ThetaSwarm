// include 3rd party libs
#include "../3rdParty/SPSCQueue/include/rigtorp/SPSCQueue.h"
#include "../3rdParty/rapidjson/filereadstream.h"
#include "../3rdParty/rapidjson/rapidjson.h"
#include "../3rdParty/rapidjson/document.h"
#include "../3rdParty/rapidjson/istreamwrapper.h"

#include <eigen-3.3.9/Eigen/Dense>
//#include <eigen-3.3.9/Eigen/Geometry>
// TODO add precompiled headers in the cmake build
#include <vector>
#include <fstream>

struct SinglePoseVector
{
    float yaw = 0;
    float x = 0;
    float y = 0;
};

struct FullPoseState
{
    SinglePoseVector q;
    SinglePoseVector q_dot;
    SinglePoseVector q_dot_dot;
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

AllPoseStates getAllPoseStates();

Eigen::Vector3f GetCurrentPose();

void start_aruco_detection();