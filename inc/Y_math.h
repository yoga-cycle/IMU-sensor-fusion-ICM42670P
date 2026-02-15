/*
 * Basic math helper functions and operations
 */
#pragma once

#include <stdint.h>

#define FIXED_POINT_SCALE 11
#define FIXED_POINT_UNIT_MAGNITUDE (1 << FIXED_POINT_SCALE)

/* get value of sqrt(a^2 + b^2)
 * link here: https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
 */
__attribute__((section(".ram_code")))
static inline int32_t Math_sqrt_of_sum_of_squares(int32_t a, int32_t b)
{
	if(a < 0)
	{
		a *= -1;
	}

	if(b < 0)
	{
		b *= -1;
	}

	int32_t max = 0, min = 0;

	if(a > b)
	{
		max = a;
		min = b;
	}
	else
	{
		max = b;
		min = a;
	}

	int32_t z0 = max + (min >> 3);
	int32_t z1 = (56*max + 33*min) >> 6;

	if(z0 > z1)
	{
		return z0;
	}

	else
	{
		return z1;
	}
}

/*
 * Multiply two quaternions and store in result
 * result = q1 X q2
 */
__attribute__((section(".ram_code")))
static inline void Math_Quaternion_multiply(const int32_t q1[4], const int32_t q2[4], int32_t result[4])
{
	result[0] = ((q1[0]*q2[0]) - (q1[1]*q2[1]) - (q1[2]*q2[2]) - (q1[3]*q2[3])) >> FIXED_POINT_SCALE;
	result[1] = ((q1[0]*q2[1]) + (q1[1]*q2[0]) + (q1[2]*q2[3]) - (q1[3]*q2[2])) >> FIXED_POINT_SCALE;
	result[2] = ((q1[0]*q2[2]) - (q1[1]*q2[3]) + (q1[2]*q2[0]) + (q1[3]*q2[1])) >> FIXED_POINT_SCALE;
	result[3] = ((q1[0]*q2[3]) + (q1[1]*q2[2]) - (q1[2]*q2[1]) + (q1[3]*q2[0])) >> FIXED_POINT_SCALE;
}

/*
 * rotate vector v by quaternion q and store rotated-vector in result
 */
__attribute__((section(".ram_code")))
static inline void Math_Quaternion_rotate_vector(const int32_t q[4], const int32_t v[3], int32_t result[3])
{
	int32_t quaternion_equivalent_vector[4], q_conjugate[4];

	quaternion_equivalent_vector[0] = 0;
	quaternion_equivalent_vector[1] = v[0];
	quaternion_equivalent_vector[2] = v[1];
	quaternion_equivalent_vector[3] = v[2];

	q_conjugate[0] = q[0];
	q_conjugate[1] = -q[1];
	q_conjugate[2] = -q[2];
	q_conjugate[3] = -q[3];

	int32_t temp[4], result_quaternion[4];

	Math_Quaternion_multiply(q_conjugate, quaternion_equivalent_vector, temp);
	Math_Quaternion_multiply(temp, q, result_quaternion);

	result[0] = result_quaternion[1];
	result[1] = result_quaternion[2];
	result[2] = result_quaternion[3];
}

/*
 * result = v1 X v2
 */
__attribute__((section(".ram_code")))
static inline void Math_Vector_cross_product(const int32_t v1[3], const int32_t v2[3], int32_t result[3])
{
	int32_t values[3];

    values[0] = v1[1]*v2[2] - v1[2]*v2[1];
    values[1] = v1[2]*v2[0] - v1[0]*v2[2];
    values[2] = v1[0]*v2[1] - v1[1]*v2[0];

    result[0] = (values[0] >> FIXED_POINT_SCALE);
    result[1] = (values[1] >> FIXED_POINT_SCALE);
    result[2] = (values[2] >> FIXED_POINT_SCALE);
}

__attribute__((section(".ram_code")))
static inline void Math_Vector_normalize(int32_t v[3])
{
	int32_t magnitude;

	magnitude = Math_sqrt_of_sum_of_squares(v[0], v[1]);
	magnitude = Math_sqrt_of_sum_of_squares(v[2], magnitude);

	v[0] = (v[0] << FIXED_POINT_SCALE);
	v[1] = (v[1] << FIXED_POINT_SCALE);
	v[2] = (v[2] << FIXED_POINT_SCALE);

	v[0] = v[0]/magnitude;
	v[1] = v[1]/magnitude;
	v[2] = v[2]/magnitude;
}

__attribute__((section(".ram_code")))
static inline void Math_Quaternion_normalize(int32_t q[4])
{
	int32_t magnitude;

	magnitude = Math_sqrt_of_sum_of_squares(q[0], q[1]);
	magnitude = Math_sqrt_of_sum_of_squares(q[2], magnitude);
	magnitude = Math_sqrt_of_sum_of_squares(q[3], magnitude);

	q[0] = (q[0] << FIXED_POINT_SCALE);
	q[1] = (q[1] << FIXED_POINT_SCALE);
	q[2] = (q[2] << FIXED_POINT_SCALE);
	q[3] = (q[3] << FIXED_POINT_SCALE);

	q[0] = q[0]/magnitude;
	q[1] = q[1]/magnitude;
	q[2] = q[2]/magnitude;
	q[3] = q[3]/magnitude;
}
