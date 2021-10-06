#include "kinematics.h"
#include "../../util/util.h"
#include <stdio.h>
#include <iostream>
#include <tuple>

using namespace Eigen;

#define PI 3.14159265

MatrixXf initialize_H_0_R(RobotConfiguration configuration)
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

// normalize wheel speed ratio with respects to 
Vector4f applySpeedConstraints(Vector4f speeds_vector, RobotConfiguration *robot_config)
{   
    // finds lowest and highest absolute values in speed vector
    // to extract the speed ratios to then multiply by proportional control
    auto [min_input_speed, max_input_speed] = findMaxMinAbsValues(&speeds_vector);

    // std::cout << "the max speeds: " << max_input_speed << " " << min_input_speed << std::endl;

    Vector4f normalized_speeds(4,1);

    for(int i = 0; i < 4; i++)
    {
        normalized_speeds(i,0) = getSign(speeds_vector(i,0)) * MapValueToRange(min_input_speed, robot_config->Min_RadS_Speed, max_input_speed+1, robot_config->Max_RadS_Speed, abs(speeds_vector(i,0)) );
    }

    //std::cout << "the normalized speeds: " << normalized_speeds << std::endl;

    return normalized_speeds;
}

// modify speeds_and_dirrections with mapped radians per second to the appropriate driving PWM duty cycle and then check constraints, 
// minimum responsive PWM is 160, max is 255, 
 VectorXi MapRadiansToPWM(Vector4f speed_vector, RobotConfiguration *robot_config)
 {
    float unit_speed_per_pwm = robot_config->Viable_PWM_Range/robot_config->Rotation_Speed_Range;
    
    VectorXi pwm_and_direction_vector(8,1); // preparing a buffer to send

    //std::cout << "The speeds vector" << speed_vector << std::endl;

    //std::cout << "unit_speed_per_pwm" << unit_speed_per_pwm << std::endl;

    Vector4f fixed_speed_vector = applySpeedConstraints(speed_vector, robot_config);

    // std::cout << "applied speed constraints in kinematics in MapRadiansToPWM" << std::endl;

    //std::cout << "The fixed_speed_vector vector" << std::endl << fixed_speed_vector << std::endl;
    
    for(uint8_t i = 0; i < 4; i++) {
        //printVector(&speed_vector, "speed_vector");

        if (abs(fixed_speed_vector(i,0)) > 0) {
            uint8_t speed_in_pwm = MapValueToRange(robot_config->Min_RadS_Speed, robot_config->Min_Viable_PWM, robot_config->Max_RadS_Speed, robot_config->Max_Viable_PWM, abs(fixed_speed_vector(i,0)) );
            
            //printf("The speed_in_pwm is: %f\n", speed_in_pwm);
            if(fixed_speed_vector(i,0) >= 0) {
                pwm_and_direction_vector(i*2, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 1, 0) = 1;
                //printVector(&pwm_and_direction_vector, "pwm_and_direction_vector");
            }
            else {
                pwm_and_direction_vector(i*2, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 1, 0) = 0;
                //printVector(&pwm_and_direction_vector, "pwm_and_direction_vector_else"); 
            }
        }
        else if(abs(fixed_speed_vector(i,0)) == 0) {
            pwm_and_direction_vector(i*2, 0) = 0;
            pwm_and_direction_vector(i*2 + 1, 0) = 0;
        }
        else {
            printf("Assigned motor speeds are INVALID\n");
        }
    }
    //std::cout << "The pwm and direction vector" << pwm_and_direction_vector << std::endl;
    return pwm_and_direction_vector;
}

// Calculates necessary PWM/Direction signal from the control vector, and modifies speeds_and_directions
bool CalculateSpeedCommands(MatrixXf H0_R, RobotConfiguration *robot_config, Vector3f control_vector, uint8_t *commands_array, int phiCurrent)
{
    Matrix3f rotation_matrix(3,3);

    rotation_matrix(0,0) = 1;
    rotation_matrix(0,1) = 0;
    rotation_matrix(0,2) = 0;
    rotation_matrix(1,0) = 0;
    rotation_matrix(1,1) = cos(phiCurrent);
    rotation_matrix(1,2) = sin(phiCurrent);
    rotation_matrix(2,0) = 0;
    rotation_matrix(2,1) = -sin(phiCurrent);
    rotation_matrix(2,2) = cos(phiCurrent);

    // std::cout << "about to calculate speed vector in kinematics" << std::endl;
    
    // speeds in radians/s of each wheel
    Vector4f speed_vector = H0_R * rotation_matrix * control_vector;

    // std::cout << "calculated speed vector in kinematics" << std::endl;
    
    // map speed-per-wheel(in radians) vector to pwm-and-direction-per-motor vector
    VectorXi speeds_and_directions_vector = MapRadiansToPWM(speed_vector, robot_config);

    // std::cout << "The speeds and directions vector \n" << speeds_and_directions_vector << std::endl;

    // std::cout << "mapped speed vector in kinematics" << std::endl;
    
    // fill in the mapped values to command array
    for(int i = 0; i < 8; i++) {
        commands_array[i + 1] = speeds_and_directions_vector(i,0);
    }

    return 1;
}