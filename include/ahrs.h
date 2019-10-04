#ifndef _AHRS_H
#define _AHRS_H

#include "arm_math.h"

/**
 * @brief AHRS data.
 */
typedef struct
{
	float32_t quaternion[4];	/**< Output quaternion describing the sensor relative to the Earth. */
	float32_t q[4];				/**< Internal quaternion describing the Earth relative to the sensor. */
	float32_t int_error[3];		/**< Tntegral error. */
	float32_t kp;				/**< Proportional gain. */
	float32_t ki;				/**< Proportional gain.integral gain. */
	float32_t sample_period;	/**< Timestamp between two packets (s). */
} arm_AHRS_f32;

/*
** AHRS functions
*/
void update_ahrs_imu(float32_t gyroscope[], float32_t accelerometer[], arm_AHRS_f32 *ahrs);

#endif /* !_AHRS_H */
