#include "kinematics.h"
#include "../../../../Common/common.h"
#include <eigen-3.3.9/Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <tuple>

using namespace Eigen;

#define PI 3.14159265

#define MAX_RAD_S_ROTATION_SPEED_1 23.3
#define MAX_RAD_S_ROTATION_SPEED_2 23.3
#define MAX_RAD_S_ROTATION_SPEED_3 23.3
#define MAX_RAD_S_ROTATION_SPEED_4 23.3

#define MIN_RAD_S_ROTATION_SPEED_1 10.5
#define MIN_RAD_S_ROTATION_SPEED_2 10.5
#define MIN_RAD_S_ROTATION_SPEED_3 10.5
#define MIN_RAD_S_ROTATION_SPEED_4 10.5

#define MAX_VIABLE_PWM 255
#define MIN_VIABLE_PWM 180

#define MAXIMUM_WHEEL_SPEEDS 23.3
#define MINIMUM_WHEEL_SPEEDS 10.5

#define ROTATION_SPEED_RANGE (MAX_RAD_S_ROTATION_SPEED_1 - MIN_RAD_S_ROTATION_SPEED_1)
#define PWM_VIABLE_RANGE (MAX_VIABLE_PWM - MIN_VIABLE_PWM)

MatrixXf H0_R(4,3);

void initialize_H_0_R()
{
    // 46(mm) / 2000 (wheel radius in meters)
    float wheel_radius_m = WHEEL_DIAMETER_MM / 2000.0;
    float matrix_scalar = 1/wheel_radius_m;

    float h_zero_00 = -(-WHEEL_TO_CENTER_X_DISTANCE_MM - WHEEL_TO_CENTER_Y_DISTANCE_MM) / 1000.0;
    float h_zero_10 = -(WHEEL_TO_CENTER_X_DISTANCE_MM + WHEEL_TO_CENTER_Y_DISTANCE_MM) / 1000.0;
    float h_zero_20 = (WHEEL_TO_CENTER_X_DISTANCE_MM + WHEEL_TO_CENTER_Y_DISTANCE_MM) / 1000.0;
    float h_zero_30 = (-WHEEL_TO_CENTER_X_DISTANCE_MM - WHEEL_TO_CENTER_Y_DISTANCE_MM) / 1000.0;
    
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

    //printf("The wheel radius is: %f \n", wheel_radius_m);
    //printf("The matrix scalar is: %f \n", matrix_scalar);
    //printMatrix(&h_zero, "h_zero");

    H0_R = matrix_scalar * h_zero;
}

Vector4f applyNonHolonomicConstraints(Vector4f speeds_vector)
{   
    // fins lowest and highest absolute values in speed vector
    // to extract the speed ratios to then multiply by proportional control
    float max_input_speed;
    float min_input_speed;
    std::tie(min_input_speed, max_input_speed) = findMaxMinAbsValues(&speeds_vector);

    //std::cout << "the max speeds: " << max_input_speed << " " << min_input_speed << std::endl;

    Vector4f normalized_speeds(4,1);

    for(int i = 0; i < 4; i++)
    {
        normalized_speeds(i,0) = getSign(speeds_vector(i,0)) * MapValueToRange(min_input_speed, MINIMUM_WHEEL_SPEEDS, max_input_speed+1, MAXIMUM_WHEEL_SPEEDS, abs(speeds_vector(i,0)) );
    }

    //std::cout << "the normalized speeds: " << normalized_speeds << std::endl;

    return normalized_speeds;
}

// modify speeds_and_dirrections with mapped radians per second to the appropriate driving PWM duty cycle and then check constraints, 
// minimum responsive PWM is 160, max is 255, 
 VectorXi MapRadiansToPWM(Vector4f speed_vector)
 {
    float unit_speed_per_pwm = PWM_VIABLE_RANGE/(ROTATION_SPEED_RANGE);
    
    VectorXi pwm_and_direction_vector(9,1); // preparing a buffer to send
    pwm_and_direction_vector(0,0) = 1; // set 0th(parse) bit as 1

    //std::cout << "The speeds vector" << speed_vector << std::endl;

    //std::cout << "unit_speed_per_pwm" << unit_speed_per_pwm << std::endl;

    Vector4f fixed_speed_vector = applyNonHolonomicConstraints(speed_vector);

    //std::cout << "The fixed_speed_vector vector" << std::endl << fixed_speed_vector << std::endl;
    
    for(uint8_t i = 0; i < 4; i++) {
        //printVector(&speed_vector, "speed_vector");

        if (abs(fixed_speed_vector(i,0)) > 0) {
            uint8_t speed_in_pwm = MapValueToRange(MINIMUM_WHEEL_SPEEDS, MIN_VIABLE_PWM, MAXIMUM_WHEEL_SPEEDS, MAX_VIABLE_PWM, abs(fixed_speed_vector(i,0)) );
            
            //printf("The speed_in_pwm is: %f\n", speed_in_pwm);
            if(fixed_speed_vector(i,0) >= 0) {
                pwm_and_direction_vector(i*2 + 1, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 2, 0) = 1;
                //printVector(&pwm_and_direction_vector, "pwm_and_direction_vector");
            }
            else {
                pwm_and_direction_vector(i*2 + 1, 0) = speed_in_pwm;
                pwm_and_direction_vector(i*2 + 2, 0) = 0;
                //printVector(&pwm_and_direction_vector, "pwm_and_direction_vector_else"); 
            }
        }
        else if(abs(fixed_speed_vector(i,0)) == 0) {
            pwm_and_direction_vector(i*2 + 1, 0) = 0;
            pwm_and_direction_vector(i*2 + 2, 0) = 0;
        }
        else {
            printf("Assigned motor speeds are INVALID\n");
        }
    }
    //std::cout << "The pwm and direction vector" << pwm_and_direction_vector << std::endl;
    return pwm_and_direction_vector;
}

// Calculates necessary PWM/Direction signal from the control vector, and modifies speeds_and_directions
bool CalculateMotorSpeedVector(Vector3f control_vector, uint8_t *speeds_and_directions, int phiCurrent){

    // TODO maybe remove this *100 shit
    // double phi = *phiCurrent * 1.00;

    // phi = phi/100;

    //phi = phi * (PI / 180.0);

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
    
    // speeds in radians/s of each wheel
    Vector4f speed_vector = H0_R * rotation_matrix * control_vector;
    
    // TODO change the creation of the parse byte
    speeds_and_directions[0] = 1;
    
    // map speed-per-wheel(in radians) vector to pwm-and-direction-per-motor vector
    VectorXi speeds_and_directions_vector = MapRadiansToPWM(speed_vector);
    
    for(uint8_t i = 1; i < 9; i++) {
        speeds_and_directions[i] = speeds_and_directions_vector(i,0);
    }

    return 1;
}