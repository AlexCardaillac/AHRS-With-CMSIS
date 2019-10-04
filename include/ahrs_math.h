#ifndef _AHRS_MATH_H
#define _AHRS_MATH_H

#include "arm_math.h"

/*
** Math functions
*/
int8_t sign(float32_t value);
float32_t mean(uint16_t len, float32_t tab[]);
float32_t vector_norm(uint16_t len, float32_t vec[]);
void vector_cross3(float32_t a[], float32_t b[], float32_t res[]);

#endif /* !_AHRS_MATH_H */
