#include "kinematics.h"
#include "../../util/util.h"
#include <stdio.h>
#include <iostream>
#include <tuple>

using namespace Eigen;

#define PI 3.14159265

// #define MAX_ROT_SPEED_OUT 6400.0

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

// modify speeds_and_dirrections with mapped radians per second to the appropriate driving PWM duty cycle and then check constraints, 
// minimum responsive PWM is 160, max is 255
// TODO rethink the -100 <-> 100 scaling to input into speed calculator, remapped here to pwm using MAX_ROT_SPEED_OUT derived empirically
 VectorXi MapRadiansToPWM(Vector4f const &speed_vector, RobotConfiguration const &robot_config)
 {
    //float unit_speed_per_pwm = robot_config.Viable_PWM_Range/robot_config.Rotation_Speed_Range;
    
    VectorXi pwm_and_direction_vector(8,1); // preparing a buffer to send

    //std::cout << "The speeds vector" << speed_vector << std::endl;

    //std::cout << "unit_speed_per_pwm" << unit_speed_per_pwm << std::endl;

    float max_rot_speed = findMaxAbsValue(speed_vector);

    // Vector4f fixed_speed_vector = applySpeedConstraints(speed_vector, robot_config);

    // std::cout << "applied speed constraints in kinematics in MapRadiansToPWM" << std::endl;

    //std::cout << "The fixed_speed_vector vector" << std::endl << fixed_speed_vector << std::endl;
    
    for(uint8_t i = 0; i < 4; i++) {
        //printVector(&speed_vector, "speed_vector");

        if (abs(speed_vector(i,0)) > 0) {
            uint8_t speed_in_pwm = MapValueToRange(0, robot_config.Min_Viable_PWM, max_rot_speed, robot_config.Max_Viable_PWM, abs(speed_vector(i,0)) );
            
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

// Calculates necessary PWM/Direction signal from the control vector, and modifies speeds_and_directions
VectorXi CalculateSpeedCommands(MatrixXf const &H0_R, RobotConfiguration const &robot_config, Vector3f const &control_vector, float phiCurrent)
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

    // std::cout << "about to calculate speed vector in kinematics" << std::endl;
    
    // speeds in radians/s of each wheel
    Vector4f speed_vector = H0_R * rotation_matrix * control_vector;

    std::cout << "calculated speed vector in kinematics: " << std::endl;
    std::cout << speed_vector << std::endl;
    std::cout << "The HOR matrix: " << std::endl;
    std::cout << H0_R << std::endl;
    std::cout << "The rotation matrix: " << std::endl;
    std::cout << rotation_matrix << std::endl;
    // std::cout << "The control vector " << std::endl;
    // std::cout << control_vector << std::endl;

    // map speed-per-wheel(in radians) vector to pwm-and-direction-per-motor vector
    VectorXi speeds_and_directions_vector = MapRadiansToPWM(speed_vector, robot_config);

    return speeds_and_directions_vector;

    // std::cout << "The speeds and directions vector \n" << speeds_and_directions_vector << std::endl;

    // std::cout << "mapped speed vector in kinematics" << std::endl;
    
    // fill in the mapped values to command array
    // for(int i = 0; i < 8; i++) {
    //     commands_array[i + 1] = speeds_and_directions_vector(i,0);
    // }

    // return 1;
}