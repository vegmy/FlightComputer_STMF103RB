#ifndef __QUATERNIONS_H_
#define __QUATERNIONS_H_

typedef struct {
    double w;
    double x;
    double y;
    double z;
} quaternion_t;

quaternion_t create_quaternion(float w, float x, float y, float z);
quaternion_t add_quaternion(quaternion_t q1, quaternion_t q2);
quaternion_t subtract_quaternion(quaternion_t q1, quaternion_t q2); 
quaternion_t multiply_quaternions(quaternion_t q1, quaternion_t q2); 
quaternion_t normalize_quaternion(quaternion_t q);


#endif