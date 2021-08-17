#include "../MotorCommands/sendMotorCommands.h"
#include "../CommonFunctions/common.h"

#include <eigen-3.3.9/Eigen/Dense>

using namespace Eigen;

void SendVectorControl(VectorXi control_input, int *current_phi, udp_client *myClient);

void run_feedback_control();