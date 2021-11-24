#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Eigen/Dense>

using namespace Eigen;

// Struct holding robot configuration data
struct RobotConfiguration {

    float Wheel_Diameter_mm = 0;
    float Wheel_To_CenterX_mm = 0;
    float Wheel_To_CenterY_mm = 0;

    int Max_Viable_PWM = 0; 
    int Min_Viable_PWM = 0;

    float Max_Controller_Speed = 0; 
    float Min_Controller_Speed = 0;

    float Max_Bucket_Tilt = 0;
    float Min_Bucket_Tilt = 0;

    float Max_Bucket_Extend = 0;
    float Min_Bucket_Extend = 0;

    int Viable_PWM_Range = 0;
    
    void setViablePWM(){
        Viable_PWM_Range = Max_Viable_PWM - Min_Viable_PWM;
    }
};

// Struct holding the robot constraints and margins
struct RobotConstraints {

    float Target_Precision_Margin_Yaw = 0;
    float Target_Precision_Margin_X = 0;
    float Target_Precision_Margin_Y = 0;

    float Error_Limit_Max_Yaw = 0;
    float Error_Limit_Max_X = 0;
    float Error_Limit_Max_Y = 0;
};

MatrixXf initialize_H_0_R(RobotConfiguration const &configuration);
VectorXi CalculateSpeedCommands(MatrixXf const &H0_R, RobotConfiguration const &robot_config, Vector3f const &control_vector, float phiCurrent, float speed_coeff);
Vector4f CalculateSpeedVector(MatrixXf const &H0_R, Vector3f const &control_vector, float phiCurrent);

#endif //ROBOT_KINEMATICS_H