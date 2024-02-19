#include "vector.h"
#include "math.h"

float vector_length(vector_t *vector)
{
    float length = (float)(sqrt(vector->x * vector->x
                              + vector->y * vector->y
                              + vector->z * vector->z));
    return length;
}

void normalize_vector(vector_t *vector)
{
    float length = vector_length(vector);
    if (length < 0.0001f) 
        length = 0.0001f;
    else 
    {
        vector->x /= length;
        vector->y /= length;
        vector->z /= length;
    }
}

vector_t cross_product(const vector_t *v1, const vector_t *v2) 
{
    vector_t result;
    result.x = v1->y * v2->z - v1->z * v2->y;
    result.y = v1->z * v2->x - v1->x * v2->z;
    result.z = v1->x * v2->y - v1->y * v2->x;
    return result;
}

float dot_product(const vector_t *v1, const vector_t *v2) 
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

