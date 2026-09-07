/*
 * kalman_1d.c
 *
 *  Created on: 6 Eyl 2026
 *      Author: furkan
 */

#include "kalman_1d.h"

#include <stddef.h>

/**
 * @brief Initializes a scalar 1D Kalman filter.
 *
 * @param filter               Pointer to filter instance.
 *
 * @param process_noise_rate   Process noise variance per second.
 *
 * @param measurement_noise    Measurement noise variance.
 *
 * @param initial_covariance   Initial state uncertainty.
 *
 * @retval 1U Success.
 * @retval 0U Invalid parameter.
 */
uint8_t Kalman1D_Init(Kalman1D_t *filter,float process_noise_rate,float measurement_noise,float initial_covariance)
{
    if (filter == NULL)
    {
        return 0U;
    }


    if ((process_noise_rate < 0.0f) || (measurement_noise <= 0.0f) || (initial_covariance < 0.0f))
    {
        return 0U;
    }

    filter->state_estimate = 0.0f;
    filter->error_covariance = initial_covariance;

    filter->process_noise_rate = process_noise_rate;
    filter->measurement_noise = measurement_noise;
    filter->initial_covariance = initial_covariance;

    filter->initialized = 0U;

    return 1U;
}


/**
 * @brief Resets the filter state while preserving Q, R and P0.
 *
 * @param filter Pointer to filter instance.
 */
void Kalman1D_Reset(Kalman1D_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->state_estimate = 0.0f;
    filter->error_covariance = filter->initial_covariance;
    filter->initialized = 0U;
}

/**
 * @brief Processes one scalar measurement.
 *
 * @param filter          Pointer to filter instance.
 *
 * @param measurement     New sensor measurement.
 *
 * @param dt_s            Time since previous measurement in seconds.
 *
 * @param filtered_value  Filtered result.
 *
 * @retval 1U Success.
 * @retval 0U Invalid parameter.
 */
uint8_t Kalman1D_Update(Kalman1D_t *filter,float measurement,float dt_s,float *filtered_value)
{
    float predicted_state;
    float predicted_covariance;
    float innovation;
    float innovation_covariance;
    float kalman_gain;
    float correction_factor;

    if ((filter == NULL) || (filtered_value == NULL))
    {
        return 0U;
    }

    if (dt_s < 0.0f)
    {
        return 0U;
    }

    /*
     * Initialize state from the first valid measurement to avoid an artificial transient from zero.
     */
    if (filter->initialized == 0U)
    {
        filter->state_estimate = measurement;
        filter->error_covariance = filter->initial_covariance;

        filter->initialized = 1U;

        *filtered_value = measurement;

        return 1U;
    }

    /*
     * Prediction
     *
     * A = 1
     * x- = x
     * P- = P + Q * dt
     */
    predicted_state = filter->state_estimate;

    predicted_covariance = filter->error_covariance + (filter->process_noise_rate * dt_s);

    /*
     * Measurement innovation
     *
     * H = 1
     * innovation = z - x-
     */
    innovation =  measurement -  predicted_state;

    innovation_covariance =  predicted_covariance +  filter->measurement_noise;


    if (innovation_covariance <= 0.0f)
    {
        return 0U;
    }

    /*
     * Kalman gain
     *
     * K = P- / (P- + R)
     */
    kalman_gain =  predicted_covariance / innovation_covariance;



    /*
      * State correction
      *
      * x = x- + K(z - x-)
      */
    filter->state_estimate = predicted_state + (kalman_gain * innovation);



    /*
     * Joseph-form covariance update
     *
     * P = (1-K)^2 * P- + K^2 * R
     */
    correction_factor = 1.0f - kalman_gain;

    filter->error_covariance =  (correction_factor *  correction_factor *  predicted_covariance) + (kalman_gain *  kalman_gain * filter->measurement_noise);

    *filtered_value = filter->state_estimate;

    return 1U;
}

