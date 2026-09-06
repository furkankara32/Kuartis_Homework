/*
 * heading.h
 *
 *  Created on: 6 Eyl 2026
 *      Author: furkan
 */

#ifndef BNO085_INC_HEADING_H_
#define BNO085_INC_HEADING_H_

#include "stdint.h"

uint8_t Heading_Calculate(float x_uT, float y_uT, float *heading_deg);

#endif /* BNO085_INC_HEADING_H_ */
