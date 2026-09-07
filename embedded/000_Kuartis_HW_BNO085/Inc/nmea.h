/*
 * nmea.h
 *
 *  Created on: 7 Eyl 2026
 *      Author: furkan
 */

#ifndef NMEA_H_
#define NMEA_H_

#include <stddef.h>

#define NMEA_HDM_BUFFER_SIZE    32U

size_t NMEA_FormatHDM(float heading_deg,char *buffer, size_t buffer_size);

#endif /* NMEA_H_ */
