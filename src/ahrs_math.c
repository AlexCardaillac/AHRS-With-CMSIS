#include "arm_math.h"

/**
 * @brief Get the sign of a value
 *        This function can be Inline but don't know how to adapt it to arm
 *
 * @param value Value to check
 * @return int8_t Get -1 if negative number, 0 if null or 1 if positive number
 */
int8_t sign(float32_t value)
{
	return value < 0 ? -1 : value > 0;
}

/**
 * @brief Compute mean value of a float32_t array
 *
 * @param len Array length
 * @param tab Array
 * @return float32_t The mean value
 */
float32_t mean(uint16_t len, float32_t tab[]) {
    float32_t sum = 0;
    for (uint16_t i = 0; i < len; i += 1)
        sum += tab[i];
    return (sum / len);
}

/**
 * @brief Compute vector norm
 *
 * @param len Vector length
 * @param vec Vector
 * @return float32_t The norm value
 */
float32_t vector_norm(uint16_t len, float32_t vec[])
{
    float32_t sum = 0.;
    for (uint16_t i = 0; i < len; i += 1) {
        sum += vec[i] * vec[i];
    }
	float32_t norm;
	arm_sqrt_f32(sum, &norm);
	return norm;
}

/**
 * @brief Compute cross product between a vector a and a vector b
 *        x = a.y * b.z - a.z * b.y
 *        y = a.z * b.x - a.x * b.z
 *        z = a.x * b.y - a.y * b.x
 *
 * @param a First Vector
 * @param b Second Vector
 * @param res Output Vcetor
 */
void vector_cross3(float32_t a[], float32_t b[], float32_t res[])
{
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}
