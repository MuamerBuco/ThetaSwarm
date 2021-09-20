#include <iostream>
#include <tuple>
#include <Eigen/Dense>

void msDelay(uint16_t milliseconds);
void PrintBuffer(uint8_t *buffer);

float MapValueToRange(float in_min, float out_min, float in_max, float out_max, float input);
int getSign(float number);
std::tuple<float, float> findMaxMinAbsValues(Eigen::Vector4f *speeds_vector);