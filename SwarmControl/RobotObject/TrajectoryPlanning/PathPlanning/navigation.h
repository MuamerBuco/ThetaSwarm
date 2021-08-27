#include <eigen-3.3.9/Eigen/Dense>
#include <vector>

struct TargetPose
{
    float x;
    float y;
    float p;
    int id;
};

struct AllTargetPoses
{
    std::vector<TargetPose> allPosesVector;
};

AllTargetPoses getAllTargetPosesAndIDs();
void fill_target_pose_buffer();