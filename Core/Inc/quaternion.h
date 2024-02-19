#ifndef __QUATERNIONS_H_
#define __QUATERNIONS_H_

#include "vector.h"

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

quaternion_t create_quaternion(float w, float x, float y, float z);
quaternion_t add_quaternion(quaternion_t q1, quaternion_t q2);
quaternion_t subtract_quaternion(quaternion_t q1, quaternion_t q2); 
quaternion_t multiply_quaternions(quaternion_t q1, quaternion_t q2); 
void normalize_quaternion(quaternion_t *q);
vector_t rotateVector(quaternion_t orientation, float vx, float vy, float vz);
quaternion_t quaternionVectorRotation(quaternion_t orientation, quaternion_t vector);
quaternion_t inverse_quaternion(quaternion_t q);
void update_orientation(vector_t *vec_angular_speed, quaternion_t *quat_orientation, float dt);
vector_t rotate_vector_by_quaternion(vector_t vec_current_orientation, quaternion_t quat_rotation);
void orientation_from_accelerometer(vector_t *vec_acc, quaternion_t *quat_acc);
void quaternion_from_gyroscope(quaternion_t *quat_out, vector_t *rot_axis, float delta_time);
void rotate_orientation_quaternion(quaternion_t* quat_out, quaternion_t *quat_rot, quaternion_t *quat_in);
void quaternion_from_acceleration(quaternion_t *q, const vector_t *a);


#endif