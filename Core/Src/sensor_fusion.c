#include "sensor_fusion.h"
#include "quaternion.h"


quaternion_t quat_complementary_filter(quaternion_t quat_gyro, quaternion_t quat_acc, float alpha)
{
    quat_gyro.w *= alpha;
    quat_gyro.x *= alpha;
    quat_gyro.y *= alpha;
    quat_gyro.z *= alpha;

    quat_acc.w *= (1 - alpha);
    quat_acc.x *= (1 - alpha);
    quat_acc.y *= (1 - alpha);
    quat_acc.z *= (1 - alpha);

    quaternion_t quat_fused = add_quaternion(quat_gyro, quat_acc);
    
    normalize_quaternion(&quat_fused);
    return quat_fused;
}