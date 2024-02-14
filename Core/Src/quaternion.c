#include <math.h>
#include <quaternion.h>

// Create a quaternion_t
quaternion_t create_quaternion(float w, float x, float y, float z)
{
    quaternion_t q = {w, x, y, z};
    return q;
}

// Add two quaternion_ts
quaternion_t add_quaternion(quaternion_t q1, quaternion_t q2) 
{
    quaternion_t result = {
        q1.w + q2.w,
        q1.x + q2.x,
        q1.y + q2.y,
        q1.z + q2.z
    };
    return result;
}

// Subtract two quaternion_ts
quaternion_t subtract_quaternion(quaternion_t q1, quaternion_t q2) 
{
    quaternion_t result = {
        q1.w - q2.w,
        q1.x - q2.x,
        q1.y - q2.y,
        q1.z - q2.z
    };
    return result;
}

// Multiply two quaternion_ts
quaternion_t multiply_quaternions(quaternion_t q1, quaternion_t q2) 
{
    quaternion_t result = {
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
    return result;
}

// Normalize a quaternion_t
quaternion_t normalize_quaternion(quaternion_t q) 
{
    float norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    quaternion_t result = {
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm
    };
    return result;
}

/*To update the orientation with quaternions after converting gyroscope data to radians per second and 
integrating it over time, you would typically follow these steps in your code:

Define a Quaternion Update Function: This function takes the gyroscope data (in radians per second) and 
integrates it over the given time period (deltaTime) to update the orientation quaternion.
The function for updating orientation with quaternions uses the data in radians per second, 
which should be the filtered data from the IMU for optimal results. Filtering the raw data from 
the gyroscope (degrees per second) through an IIR filter and then converting it to radians per second helps 
reduce noise and improve the accuracy of the orientation estimation. This preprocessing step is crucial 
for maintaining the integrity of the orientation data, especially in dynamic environments where IMU readings 
can be affected by various sources of noise.
*/

void quaternionUpdate(float gx, float gy, float gz, float deltaTime, float* q) {
    // Half the gyroscope values for integration
    gx *= 0.5f * deltaTime;
    gy *= 0.5f * deltaTime;
    gz *= 0.5f * deltaTime;

    // Current quaternion values
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Quaternion integration using gyroscope data
    q[0] += (-q1 * gx - q2 * gy - q3 * gz);
    q[1] += (q0 * gx + q2 * gz - q3 * gy);
    q[2] += (q0 * gy - q1 * gz + q3 * gx);
    q[3] += (q0 * gz + q1 * gy - q2 * gx);

    // Normalize the quaternion
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}
