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

