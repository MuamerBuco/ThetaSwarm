#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Eigen/Dense>

using namespace Eigen;

#define MIN_NORM_SPEED -35.0
#define MAX_NORM_SPEED 35.0

// struct holding robot configuration data
struct RobotConfiguration {

    float Wheel_Diameter_mm;
    float Wheel_To_CenterX_mm;
    float Wheel_To_CenterY_mm;

    int Max_Viable_PWM; 
    int Min_Viable_PWM;

    float Max_RadS_Speed; 
    float Min_RadS_Speed;

    float Max_Bucket_Tilt;
    float Min_Bucket_Tilt;

    float Max_Bucket_Extend;
    float Min_Bucket_Extend;

    float Rotation_Speed_Range = (Max_RadS_Speed - Min_RadS_Speed);
    int Viable_PWM_Range = (Max_Viable_PWM - Min_Viable_PWM);
};

// struct holding the robot constraints and margins
struct RobotConstraints {

    float Target_Precision_Margin_Yaw;
    float Target_Precision_Margin_X;
    float Target_Precision_Margin_Y;
};

MatrixXf initialize_H_0_R(RobotConfiguration const &configuration);
VectorXi CalculateSpeedCommands(MatrixXf const &H0_R, RobotConfiguration const &robot_config, Vector3f const &control_vector, float phiCurrent);

#endif //ROBOT_KINEMATICS_H