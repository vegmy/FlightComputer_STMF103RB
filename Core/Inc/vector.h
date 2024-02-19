#ifndef __VECTOR_H
#define __VECTOR_H

typedef struct
{
    float x;
    float y;
    float z;
} vector_t;

float vector_length(vector_t *vector);
void normalize_vector(vector_t *vector);
vector_t cross_product(const vector_t *v1, const vector_t *v2);
float dot_product(const vector_t *v1, const vector_t *v2);

#endif