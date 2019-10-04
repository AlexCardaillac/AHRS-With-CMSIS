#include "ahrs.h"
#include "6dof_math.h"

/**
 * @brief Calculate conjugate of quaternion
 *
 * @param q Quaternion to conjugate
 * @param dest Storage variable
 */
void quatern_conj(float32_t q[], float32_t dest[])
{
    dest[0] = q[0];
	dest[1] = -q[1];
	dest[2] = -q[2];
	dest[3] = -q[3];
}

/**
 * @brief Set the quaternion variable
 *
 * @param ahrs Algorithm structure
 * @param value Value to compute
 */
void set_quaternion(arm_AHRS_f32 *ahrs, float32_t value[])
{
	float32_t norm = vector_norm(4, value);
	if (norm == 0) {
		/* This part should be modified to correctly notify error
		Quaternion magnitude cannot be zero. */
		return;
	}
	arm_scale_f32(value, 1/norm, value, 4);
	memcpy(ahrs->quaternion, value, sizeof(float32_t) * 4);
	quatern_conj(value, ahrs->q);
}

/**
 * @brief Product of two quaternions
 *
 * @param a First quaternion to multiply
 * @param b Second quaternion to multiply
 * @param c Product result
 */
void quatern_prod(float32_t a[], float32_t b[], float32_t c[])
{
	c[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
	c[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
	c[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
	c[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}

/**
 * @brief Compute an iteration of the AHRS algorithm
 *
 * @param gyroscope Gyroscope data
 * @param accelerometer Accelerometer data
 * @param ahrs AHRS algorithm structure
 */
void update_ahrs_imu(float32_t gyroscope[], float32_t accelerometer[], arm_AHRS_f32 *ahrs)
{
	/* Normalise accelerometer measurement */
	float32_t norm = vector_norm(3, accelerometer);
	if (norm == 0) {
		/*
		 * This part should be modified to correctly notify error
		 * raise AHRSError('Accelerometer magnitude is zero. Algorithm update aborted.')
		 */
		return;
	}
	accelerometer[0] /= norm;
	accelerometer[1] /= norm;
	accelerometer[2] /= norm;

	/* Compute error between estimated and measured direction of gravity */
	float32_t v[] = {
		2*(ahrs->q[1]*ahrs->q[3] - ahrs->q[0]*ahrs->q[2]),
		2*(ahrs->q[0]*ahrs->q[1] + ahrs->q[2]*ahrs->q[3]),
		ahrs->q[0]*ahrs->q[0] - ahrs->q[1]*ahrs->q[1] - ahrs->q[2]*ahrs->q[2] + ahrs->q[3]*ahrs->q[3] /* estimated direction of gravity */
	};

	float32_t error[3];
	vector_cross3(v, accelerometer, error);
	arm_add_f32(ahrs->int_error, error, ahrs->int_error, 3); /* compute integral feedback terms (only outside of init period) */

	/* Apply feedback terms */
	float32_t ref[3];
	arm_scale_f32(error, ahrs->kp, error, 3);
	arm_scale_f32(ahrs->int_error, ahrs->ki, ref, 3);
	arm_add_f32(error, ref, ref, 3);
	arm_sub_f32(gyroscope, ref, ref, 3);

	/* Compute rate of change of quaternion */
	float32_t p_dot[4];
	quatern_prod(ahrs->q, (float32_t []){0, ref[0], ref[1], ref[2]}, p_dot);
	arm_scale_f32(p_dot, 0.5, p_dot, 4);
	/* integrate rate of change of quaternion */
	arm_scale_f32(p_dot, ahrs->sample_period, p_dot, 4);
	arm_add_f32(ahrs->q, p_dot, ahrs->q, 4);
	/* normalise quaternion */
	arm_scale_f32(ahrs->q, 1/vector_norm(4, ahrs->q), ahrs->q, 4);

	/* Store conjugate */
	quatern_conj(ahrs->q, p_dot);
	set_quaternion(ahrs, p_dot);
}
