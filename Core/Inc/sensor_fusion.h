#ifndef __SENSOR_FUSION_H
#define __SENSOR_FUSION_H

#include "quaternion.h"

quaternion_t quat_complementary_filter(quaternion_t quat_gyro, quaternion_t quat_acc, float alpha);

#endif