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

    for(int i = 0; i < sizeof(buffer); i++)
    {
        std::cout << unsigned(*buffer++) << " ";
    }

    std::cout << std::endl;
}

std::tuple<float, float> findMaxMinAbsValues(Vector4f *speeds_vector)
{
    Vector4f speeds_vector_copy = *speeds_vector;

    for(int i = 0; i < speeds_vector_copy.size(); i++) {
        speeds_vector_copy(i,0) = abs(speeds_vector_copy(i,0));
    }

    std::sort(speeds_vector_copy.data(), speeds_vector_copy.data() + speeds_vector_copy.size());

    return { speeds_vector_copy[0], speeds_vector_copy[3] };
}

// map value to range, crop if out of range
float MapValueToRange(float in_min, float out_min, float in_max, float out_max, float input)
{

    // if(input > in_max) input = in_max;
    // if(input < in_min) input = in_min;

    // std::cout << "///////////////////////////////" << std::endl;
    
    // std::cout << "The in_min: " << in_min << std::endl;
    // std::cout << "The in_max: " << in_max << std::endl;
    // std::cout << "The out_min: " << out_min << std::endl;
    // std::cout << "The out_max: " << out_max << std::endl;
    // std::cout << "The input: " << input << std::endl;

    float slope = 1.0 * (out_max - out_min) / (in_max - in_min);
    float output = out_min + slope * (input - in_min);

    // std::cout << "The output: " << output << std::endl;
    // std::cout << "///////////////////////////////" << std::endl;

    return output;
}

int getSign(float number)
{
    if(number < 0) return -1;
    else return 1;
}