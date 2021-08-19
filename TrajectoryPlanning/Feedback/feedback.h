#include "../../RobotCommands/sendMotorCommands.h"
#include "../../Common/common.h"

#include <eigen-3.3.9/Eigen/Dense>

using namespace Eigen;

void SendVectorControl(Vector3f control_input, int *current_phi, udp_client *myClient);

void run_feedback_control();