#include "util.h"
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace Eigen;

void msDelay(uint16_t milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void PrintBuffer(uint8_t *buffer)
{
    std::cout << "Buffer contents: " << std::endl;

    for(int i = 0; i < 12; i++)
    {
        std::cout << unsigned(*buffer++) << " ";
    }

    std::cout << std::endl;
}

namespace Eigen 
{
    auto begin(Vector4f const &m) { return m.data(); }
    auto begin(Vector3f const &m) { return m.data(); }

    auto end(Vector4f const &m) { return m.data()+m.size(); }
    auto end(Vector3f const &m) { return m.data()+m.size(); }
}


float findMaxAbsValue(Vector4f const &input_vector)
{
    Vector4f *iter = NULL;

    float max_value = 0;

    for (auto it = Eigen::begin(input_vector); it != Eigen::end(input_vector); ++it) 
    {
        if( max_value < abs(*it) ) max_value = abs(*it);
    }

    return max_value;
}

float findMaxAbsValue(Vector3f const &input_vector)
{
    Vector3f *iter = NULL;

    float max_value = 0;

    for (auto it = Eigen::begin(input_vector); it != Eigen::end(input_vector); ++it) 
    {
        if( max_value < abs(*it) ) max_value = abs(*it);
    }

    return max_value;
}

float MapValueToRange(float in_min, float out_min, float in_max, float out_max, float input)
{
    float slope = 1.0 * (out_max - out_min) / (in_max - in_min);
    float output = out_min + slope * (input - in_min);

    return output;
}

int getSign(float number)
{
    if(number < 0) return -1;
    else return 1;
}