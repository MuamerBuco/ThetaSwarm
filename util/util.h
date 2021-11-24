#ifndef ROBOT_UTIL_H
#define ROBOT_UTIL_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <atomic> 

// Error codes
#define FATAL_ERROR 1
#define CASE_ERROR 2

// struct holding field data
struct FieldData {
    
    float Max_Input_Value_P = 0;
    float Min_Input_Value_P = 0;
    float Max_Output_Value_P = 0;
    float Min_Output_Value_P = 0;

    float Max_Input_Value_X = 0;
    float Min_Input_Value_X = 0;
    float Max_Output_Value_X = 0;
    float Min_Output_Value_X = 0;

    float Max_Input_Value_Y = 0;
    float Min_Input_Value_Y = 0;
    float Max_Output_Value_Y = 0;
    float Min_Output_Value_Y = 0;

    int Number_of_units = 0;
    
    bool dataLoaded = false;
};

struct CameraSettings {
    
    int VideoPath = 0;

    int VideoWidth = 0;
    int VideoHeight = 0;

    float Contrast = 0;
    float Brightness = 0;
    float Saturation = 0;
    float Gain = 0;
    float Gamma = 0;
    float Sharpness = 0;
};

struct ArucoParams {
    
    float minMarkerPerimeterRate = 0.03;
    float maxMarkerPerimeterRate = 4;
    float adaptiveThreshWinSizeMin = 3;
    float adaptiveThreshWinSizeMax = 23;
    float adaptiveThreshWinSizeStep = 10;
};

struct SinglePose {

    float yaw = 0;
    float x = 0;
    float y = 0;
};

struct FullPoseState {

    SinglePose q;
    SinglePose q_dot;
    SinglePose q_dot_dot;
};

struct ChassisFullState {

    int id;
    FullPoseState pose_state;
};

struct AllPoseStates {

    std::vector<ChassisFullState> poses;
};

// struct holding bucket state(tilt, extension)
struct BucketState {
    
    float tilt = 0;
    float extension = 0;
};

enum LEDProgram {
    
    NONE,
    BLINK_RED,
    SOLID_RED,
    SOLID_BLUE,
    SOLID_GREEN,
};

enum CustomLEDprograms {

	SET_SINGLE_PIXEL = 1,
	SET_ALL_PIXELS,
	RAINBOW,
	THEATER_CHASE,
	FADE_IN_OUT,
    BLINK_ONCE,
    BLINK_N_TIMES
};

// struct holding LED ring state(running program)
struct SignalLED {

    LEDProgram program = NONE;
};

struct SingleStateTrajectory {

    SinglePose pose = {0, 0, 0};
    SignalLED LED_state;
    BucketState bucket_state;
};

typedef std::vector<SingleStateTrajectory> FullStateTrajectory;

// struct holding the current robot state(pose, LED and bucket)
struct FullRobotState {
    
    ChassisFullState pose_and_id;

    SignalLED LED_state;
    BucketState bucket_state;
};

void msDelay(uint16_t milliseconds);
void PrintBuffer(uint8_t *buffer);

float MapValueToRange(float in_min, float out_min, float in_max, float out_max, float input);
int getSign(float number);
float findMaxAbsValue(Eigen::Vector4f const &speeds_vector);
float findMaxAbsValue(Eigen::Vector3f const &speeds_vector);

#endif // ROBOT_UTIL_H