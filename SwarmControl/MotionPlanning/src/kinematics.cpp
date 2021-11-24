#include "../kinematics.h"
#include "../../../util/util.h"
#include <stdio.h>

using namespace Eigen;

#define PI 3.14159265

MatrixXf initialize_H_0_R(RobotConfiguration const &configuration)
{
    // wheel diameter mm / 2000 (wheel radius in meters)
    float wheel_radius_m = configuration.Wheel_Diameter_mm / 2000.0;
    float matrix_scalar = 1/wheel_radius_m;

    float h_zero_00 = -(-configuration.Wheel_To_CenterX_mm - configuration.Wheel_To_CenterY_mm) / 1000.0;
    float h_zero_10 = -(configuration.Wheel_To_CenterX_mm + configuration.Wheel_To_CenterY_mm) / 1000.0;
    float h_zero_20 = (configuration.Wheel_To_CenterX_mm + configuration.Wheel_To_CenterY_mm) / 1000.0;
    float h_zero_30 = (-configuration.Wheel_To_CenterX_mm - configuration.Wheel_To_CenterY_mm) / 1000.0;

    MatrixXf h_zero(4,3);
    h_zero(0,0) = h_zero_00;
    h_zero(0,1) = 1;
    h_zero(0,2) = 1;
    h_zero(1,0) = h_zero_10;
    h_zero(1,1) = 1;
    h_zero(1,2) = -1;
    h_zero(2,0) = h_zero_20;
    h_zero(2,1) = 1;
    h_zero(2,2) = -1;
    h_zero(3,0) = h_zero_30;
    h_zero(3,1) = 1;
    h_zero(3,2) = 1;

    return matrix_scalar * h_zero;
}

// Calculates the rotation speed of wheels based on input control vector using kinematic model
Vector4f CalculateSpeedVector(MatrixXf const &H0_R, Vector3f const &control_vector, float phiCurrent)
{
    Matrix3f rotation_matrix(3,3);

    rotation_matrix(0,0) = 1.0;
    rotation_matrix(0,1) = 0.0;
    rotation_matrix(0,2) = 0.0;
    rotation_matrix(1,0) = 0.0;
    rotation_matrix(1,1) = cos(phiCurrent);
    rotation_matrix(1,2) = sin(phiCurrent);
    rotation_matrix(2,0) = 0.0;
    rotation_matrix(2,1) = -sin(phiCurrent);
    rotation_matrix(2,2) = cos(phiCurrent);
    
    // speeds in radians/s of each wheel
    Vector4f speed_vector = H0_R * rotation_matrix * control_vector;

    return speed_vector;
}

// Calculates necessary PWM/Direction signal from the wheel speeds vector
VectorXi CalculateSpeedCommands(MatrixXf const &H0_R, RobotConfiguration const &robot_config, Vector3f const &control_vector, float phiCurrent, float speed_coeff)
{
    Vector4f speed_vector = CalculateSpeedVector(H0_R, control_vector, phiCurrent);

    VectorXi pwm_and_direction_vector(8,1); // preparing a buffer to send

    float max_rot_speed_this_pass = findMaxAbsValue(speed_vector);
    float speed_to_pwm = robot_config.Min_Viable_PWM + (robot_config.Viable_PWM_Range * speed_coeff);

    // std::clog << "The robot_config.Min_Viable_PWM is: " << robot_config.Min_Viable_PWM << std::endl;
    // std::clog << "The robot_config.Viable_PWM_Range: " << robot_config.Viable_PWM_Range << std::endl;
    // std::clog << "The speed coefficient is: " << speed_coeff << std::endl;
    // std::clog << "The max pwm this run is: " << speed_to_pwm << std::endl;
    // std::clog << "The speed_to_pwm this run is: " << speed_to_pwm << std::endl;

    for(uint8_t i = 0; i < 4; i++) 
    {
        if (abs(speed_vector(i,0)) > 0) {
            uint8_t speed_in_pwm = MapValueToRange(0, robot_config.Min_Viable_PWM, max_rot_speed_this_pass, speed_to_pwm, abs(speed_vector(i,0)) );
            
            if(speed_vector(i,0) >= 0) {
                pwm_and_direction_vector(i*2, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 1, 0) = 1;
            }
            else {
                pwm_and_direction_vector(i*2, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 1, 0) = 0;
            }
        }
        else if(abs(speed_vector(i,0)) == 0) {
            pwm_and_direction_vector(i*2, 0) = 0;
            pwm_and_direction_vector(i*2 + 1, 0) = 0;
        }
        else {
            std::cerr << "Assigned motor speeds are INVALID" << std::endl;
        }
    }

    return pwm_and_direction_vector;
}