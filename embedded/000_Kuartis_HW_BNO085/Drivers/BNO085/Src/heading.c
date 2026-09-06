/*
 * heading.c
 *
 *  Created on: 6 Eyl 2026
 *      Author: furkan
 */

#include "heading.h"
#include "math.h"

#define HEADING_RAD_TO_DEG     (57.2957795131f)
#define HEADING_MIN_FIELD_UT   (0.001f)


/**
 * @brief Calculates magnetic heading from X and Y magnetometer components.
 *
 * @param x_uT Magnetic field X component in microtesla.
 * @param y_uT Magnetic field Y component in microtesla.
 * @param heading_deg Pointer to calculated heading in degrees [0, 360).
 *
 * @retval 1U if heading was calculated successfully.
 * @retval 0U if input or output is invalid.
 */
uint8_t Heading_Calculate(float x_uT, float y_uT, float *heading_deg)
{
		float heading;

		if(heading_deg == NULL)
		{
			return 0U;
		}

		if ((fabsf(x_uT) < HEADING_MIN_FIELD_UT) && (fabsf(y_uT) < HEADING_MIN_FIELD_UT))
		{
			 return 0U;
		}

		 heading = atan2f(y_uT, x_uT) * HEADING_RAD_TO_DEG;

		 if (heading < 0.0f)
		 {
			 heading += 360.0f;
		 }

		 *heading_deg = heading;

		 return 1U;
}
