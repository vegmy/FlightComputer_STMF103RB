#include "math.h"
#include "math_lib.h"

const float PI = 3.14159265358979323846f;

/**
 * @brief Converts degrees per second to radians per second.
 *
 * @param dps Degrees per second.
 */
float dps_to_rps(float dps)
{
    float rps = dps * (PI / 180.0f);
    return rps;
}