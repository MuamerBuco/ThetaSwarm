// #ifndef STRING
// #define STRING
// #include <string>
// #endif

#include <eigen-3.3.9/Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#define MAX_NUMBER_OF_ACTIVE_UNITS 6

#define MAX_INPUT_VALUE_P 11
#define MIN_INPUT_VALUE_P 18
#define MAX_OUPUT_VALUE_P 10
#define MIN_OUPUT_VALUE_P 15

#define MIN_INPUT_VALUE_X -1.235
#define MAX_INPUT_VALUE_X 1.345
#define MAX_OUPUT_VALUE_X 190
#define MIN_OUPUT_VALUE_X 0

#define MIN_INPUT_VALUE_Y 0.80
#define MAX_INPUT_VALUE_Y -0.81
#define MAX_OUPUT_VALUE_Y 120
#define MIN_OUPUT_VALUE_Y 0

struct ChassisPoseAndID
{
    int yaw;
    int x;
    int y;
    int id;
};

struct AllPosesInPass
{
    std::vector<ChassisPoseAndID> poses;
};

AllPosesInPass getAllPosesAndIDs();

void start_aruco_detection();