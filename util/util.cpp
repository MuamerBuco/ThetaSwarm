#include "util.h"
#include <unistd.h>

using namespace Eigen;

// Delay N milliseconds
void msDelay(uint16_t milliseconds)
{
    usleep(milliseconds * 1000);
}

// Print buffer
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

    auto end(Vector4f const &m) { return m.data()+m.size(); }
}


float findMaxAbsValue(Vector4f const &speeds_vector)
{
    Vector4f *iter = NULL;

    float max_value = 0;

    for (auto it = Eigen::begin(speeds_vector); it != Eigen::end(speeds_vector); ++it) 
    {
        if( max_value < abs(*it) ) max_value = abs(*it);
    }

    return max_value;
}

// map value to range, crop if out of range
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