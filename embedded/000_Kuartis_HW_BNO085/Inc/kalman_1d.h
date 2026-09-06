/*
 * kalman_1d.h
 *
 *  Created on: 6 Eyl 2026
 *      Author: furkan
 */

#ifndef KALMAN_1D_H_
#define KALMAN_1D_H_

#include <stdint.h>

typedef struct
{
    float state_estimate;          /* x */
    float error_covariance;        /* P */

    float process_noise_rate;      /* Q, variance / second */
    float measurement_noise;       /* R, measurement variance */
    float initial_covariance;      /* P0 */

    uint8_t initialized;

} Kalman1D_t;

uint8_t Kalman1D_Init(Kalman1D_t *filter,float process_noise_rate,float measurement_noise,float initial_covariance);
void Kalman1D_Reset(Kalman1D_t *filter);
uint8_t Kalman1D_Update(Kalman1D_t *filter,float measurement,float dt_s,float *filtered_value);
#endif /* KALMAN_1D_H_ */
