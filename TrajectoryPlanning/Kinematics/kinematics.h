// Wheel configuration
#define WHEEL_DIAMETER_MM 46
#define WHEEL_TO_CENTER_X_DISTANCE_MM 57
#define WHEEL_TO_CENTER_Y_DISTANCE_MM 57

#include <eigen-3.3.9/Eigen/Dense>

using namespace Eigen;

void initialize_H_0_R();
int CalculateMotorSpeedVector(Vector3f control_vector, uint8_t *speeds_and_directions, int *phiCurrent);