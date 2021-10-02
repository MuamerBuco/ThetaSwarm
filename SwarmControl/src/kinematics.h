#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Eigen/Dense>

using namespace Eigen;

// struct holding robot configuration data
struct RobotConfiguration {

    float Wheel_Diameter_mm;
    float Wheel_To_CenterX_mm;
    float Wheel_To_CenterY_mm;

    float Max_RadS_RotationSpeed_LU; 
    float Max_RadS_RotationSpeed_RU;
    float Max_RadS_RotationSpeed_LD;
    float Max_RadS_RotationSpeed_RD;

    float Min_RadS_RotationSpeed_LU;
    float Min_RadS_RotationSpeed_RU;
    float Min_RadS_RotationSpeed_LD;
    float Min_RadS_RotationSpeed_RD;

    int Max_Viable_PWM; 
    int Min_Viable_PWM;

    float Max_RadS_Speed; 
    float Min_RadS_Speed;

    float Rotation_Speed_Range = (Max_RadS_RotationSpeed_LU - Min_RadS_RotationSpeed_LU);
    int Viable_PWM_Range = (Max_Viable_PWM - Min_Viable_PWM);
};

// struct holding the robot constraints and margins
struct RobotConstraints {

    float Target_Precision_Margin_Yaw;
    float Target_Precision_Margin_X;
    float Target_Precision_Margin_Y;
};

MatrixXf initialize_H_0_R(RobotConfiguration configuration);
bool CalculateSpeedCommands(MatrixXf H0_R, RobotConfiguration *configuration, Vector3f control_vector, uint8_t *speeds_and_directions, int phiCurrent);

#endif //ROBOT_KINEMATICS_H