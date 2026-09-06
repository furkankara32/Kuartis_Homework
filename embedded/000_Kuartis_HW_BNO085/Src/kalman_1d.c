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

    /*
     * Covariances cannot be negative.
     * R must be strictly greater than zero because it is used
     * in the Kalman gain denominator.
     */
    if ((process_noise_rate < 0.0f) ||
        (measurement_noise <= 0.0f) ||
        (initial_covariance < 0.0f))
    {
        return 0U;
    }

    filter->state_estimate = 0.0f;
    filter->error_covariance = initial_covariance;

    filter->process_noise_rate = process_noise_rate;
    filter->measurement_noise = measurement_noise;
    filter->initial_covariance = initial_covariance;

    filter->last_gain = 0.0f;
    filter->last_innovation = 0.0f;

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

    filter->last_gain = 0.0f;
    filter->last_innovation = 0.0f;

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
     * The absolute magnetic field value is not known before
     * the first measurement. Therefore initialize the state
     * directly from the first valid sensor sample.
     */
    if (filter->initialized == 0U)
    {
        filter->state_estimate = measurement;
        filter->error_covariance = filter->initial_covariance;

        filter->last_gain = 0.0f;
        filter->last_innovation = 0.0f;

        filter->initialized = 1U;

        *filtered_value = measurement;

        return 1U;
    }

    /*
     * Prediction
     *
     * Model:
     * x(k) = x(k-1) + process noise
     *
     * Since A = 1:
     * x_predicted = x
     */
    predicted_state = filter->state_estimate;

    /*
     * Process uncertainty grows with elapsed time.
     *
     * P_predicted = P + Q * dt
     */
    predicted_covariance =
        filter->error_covariance +
        (filter->process_noise_rate * dt_s);

    /*
     * Innovation:
     *
     * measurement - predicted measurement
     *
     * Since H = 1:
     * innovation = z - x_predicted
     */
    innovation =
        measurement -
        predicted_state;

    /*
     * Innovation covariance:
     *
     * S = P_predicted + R
     */
    innovation_covariance =
        predicted_covariance +
        filter->measurement_noise;

    if (innovation_covariance <= 0.0f)
    {
        return 0U;
    }

    /*
     * Kalman Gain:
     *
     * K = P_predicted / S
     */
    kalman_gain =
        predicted_covariance /
        innovation_covariance;

    /*
     * Measurement correction:
     *
     * x = x_predicted + K * innovation
     */
    filter->state_estimate =
        predicted_state +
        (kalman_gain * innovation);

    /*
     * Joseph-form covariance update.
     *
     * Scalar form:
     *
     * P = (1-K)^2 * P_predicted + K^2 * R
     *
     * Slightly more numerically robust than:
     * P = (1-K) * P_predicted
     */
    correction_factor = 1.0f - kalman_gain;

    filter->error_covariance =
        (correction_factor *
         correction_factor *
         predicted_covariance) +
        (kalman_gain *
         kalman_gain *
         filter->measurement_noise);

    filter->last_gain = kalman_gain;
    filter->last_innovation = innovation;

    *filtered_value = filter->state_estimate;

    return 1U;
}

