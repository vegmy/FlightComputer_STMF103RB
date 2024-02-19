#include "math.h"
#include "vector.h"
#include "quaternion.h"

// Create a quaternion_t
quaternion_t create_quaternion(float a, float i, float j, float k)
{
    quaternion_t q = {a, i, j, k};
    return q;
}

// Add two quaternions
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

// Subtract two quaternions
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

// Multiply two quaternions
quaternion_t multiply_quaternions(quaternion_t q1, quaternion_t q2) 
{
    quaternion_t result = 
    {
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
    return result;
}

// Normalize a quaternions
void normalize_quaternion(quaternion_t *q) 
{
    double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
};


quaternion_t inverse_quaternion(quaternion_t q) 
{
    float norm_squared = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    quaternion_t result = {
        q.w / norm_squared,
        -q.x / norm_squared,
        -q.y / norm_squared,
        -q.z / norm_squared
    };
    return result;
}
// produserer dq
void quaternion_from_angle_and_vector(quaternion_t* quat_out, float rad_angle, vector_t* vector) 
{
	float sin_halfangle = (float)(sin( (double)(rad_angle/2) )); // Sinus til halv vinkel, brukes senere
	vector_t norm_v = *vector;
	normalize_vector(&norm_v); // Normalisert vektor for rotasjonsakse

	quat_out->w = (float)( cos( (double)(rad_angle/2) ) ); // Beskriver vinkelen for rotasjon som et kvaterniontall ( ikke radianer eller grader )
	quat_out->x = (float)( norm_v.x*sin_halfangle );   // i, j og k beskriver aksen det roteres rundt
	quat_out->y = (float)( norm_v.y*sin_halfangle );
	quat_out->z = (float)( norm_v.z*sin_halfangle );
}

//dq fra gyro
void quaternion_from_gyroscope(quaternion_t *quat_out, vector_t *rot_axis, float delta_time)
{
    float theta = vector_length(rot_axis) * delta_time;
    quaternion_from_angle_and_vector(quat_out, theta, rot_axis);
}

// void rotate_orientation_quaterniont(quaternion_t )



void update_orientation(vector_t *vec_angular_speed, quaternion_t *quat_orientation, float dt) 
{
/*
1. Integrer lengden av vinkelhastighetsvektoren med dt for å finne θ.
2. Normaliser vinkelhastighetsvektoren for å finne rotasjonsaksen.
3. Beregn delta kvaternionen basert på θ og den normaliserte rotasjonsaksen.
4. Oppdater den nåværende orienteringskvaternionen ved å multiplisere den med delta kvaternionen.
5. Normaliser den oppdaterte orienteringskvaternionen for å bevare enhetens lengde.
*/
    float angular_speed_length = vector_length(vec_angular_speed);
    float theta = angular_speed_length * dt;  //finner vinkelen

    if (theta > 0.0f) 
    {
        // Normaliser vinkelhastighetsvektoren for å finne rotasjonsaksen
        float nx = vec_angular_speed->x / angular_speed_length;
        float ny = vec_angular_speed->y / angular_speed_length;
        float nz = vec_angular_speed->z / angular_speed_length;

        // Beregn delta kvaternionen
        quaternion_t quat_delta = {cos(theta/2), nx*sin(theta/2), ny*sin(theta/2), nz*sin(theta/2)};

        // Oppdater den nåværende orienteringskvaternionen
        *quat_orientation = multiply_quaternions(quat_delta, *quat_orientation);

        // Normaliser den oppdaterte orienteringskvaternionen
        normalize_quaternion(quat_orientation);
    }
}

vector_t rotate_vector_by_quaternion(vector_t vec_current_orientation, quaternion_t quat_rotation) {
    // Normaliser først kvaternionen for å sikre at rotasjonen er gyldig
    quaternion_t quat_vector_repr = {0.0f, vec_current_orientation.x,
                                           vec_current_orientation.y,
                                           vec_current_orientation.z};
    quaternion_t quat_invers = inverse_quaternion(quat_rotation);
    quaternion_t v_rotated = multiply_quaternions(multiply_quaternions(quat_rotation, quat_vector_repr), quat_invers);

    vec_current_orientation.x = v_rotated.x;
    vec_current_orientation.y = v_rotated.y;
    vec_current_orientation.z = v_rotated.z;

    return vec_current_orientation;
}

void quaternion_from_acceleration(quaternion_t *q, const vector_t *a)
{
    vector_t up = {0.0f, 0.0f, 1.0f}; // Målvektor
    vector_t norm_a = *a;
    normalize_vector(&norm_a); // Normaliser akselerasjonsvektoren

    vector_t axis = cross_product(&norm_a, &up); // Beregn rotasjonsaksen
    float angle = acosf(dot_product(&norm_a, &up)); // Beregn vinkelen

    // Normaliser rotasjonsaksen
    normalize_vector(&axis);

    // Konverter vinkel og akse til kvaternion
    q->w = cosf(angle / 2.0f);
    q->x = axis.x * sinf(angle / 2.0f);
    q->y = axis.y * sinf(angle / 2.0f);
    q->z = axis.z * sinf(angle / 2.0f);
}

void rotate_orientation_quaternion(quaternion_t* quat_out, quaternion_t *quat_rot, quaternion_t *quat_in) {

	quaternion_t quat_inverse_rot = inverse_quaternion(*quat_rot);
	// Utfør kvaternionmultiplikasjonen v_out = rot * v_in * rot^-1
    *quat_out = multiply_quaternions(multiply_quaternions(*quat_rot, *quat_in), quat_inverse_rot);
}
