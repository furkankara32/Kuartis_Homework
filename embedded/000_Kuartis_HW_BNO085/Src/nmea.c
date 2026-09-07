/*
 * nmea.c
 *
 *  Created on: 7 Eyl 2026
 *      Author: furkan
 */


#include "nmea.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>



/**
 * @brief Calculates the frame body checksum value
 *
 * @param Frame body
 *
 * @return checksum value
 */
static uint8_t NMEA_CalculateChecksum(const char *body)
{
	 uint8_t checksum = 0U;

	 if(body == NULL)
	 {
		 return 0;
	 }

	 while(*body != '\0')
	 {
		 checksum ^= (uint8_t)(*body);
		 body++;
	 }

	 return checksum;
}


/**
 * @brief Formats a magnetic heading as an NMEA 0183 HDM sentence.
 *
 * @param heading_deg Heading in degrees [0, 360).
 *
 * @param buffer Destination buffer.
 *
 * @param buffer_size Size of destination buffer.
 *
 * @return Number of generated characters excluding null terminator.
 *         Returns 0 on error.
 */
size_t NMEA_FormatHDM(float heading_deg,char *buffer, size_t buffer_size)
{
	char body[20];

	uint16_t heading_hundredths;
	uint16_t heading_whole;
	uint16_t heading_fraction;

	uint8_t checksum;

	int body_length;
	int sentence_length;

	if( (buffer == NULL) || (buffer_size == 0U) )
	{
		return 0U;
	}

	/* Heading degree must be between 0.00 < heading < 360.00*/
	if( (!isfinite(heading_deg)) || (heading_deg < 0.0f) || (heading_deg >= 360.0f) )
	{
		return 0U;
	}

	heading_hundredths = (uint16_t)( (heading_deg * 100.0f) );
	heading_whole = heading_hundredths / 100U;	// Calculate whole part
	heading_fraction = heading_hundredths % 100U; // Calculate fraction part

	/*
	 * NMEA Sentence = $HCHDM,98.88,M*11\r\n
	 * NMEA Sentence Body = HCHDM,98.88,M
	 * */
	body_length = snprintf(body, sizeof(body), "HCHDM,%u.%02u,M",(unsigned int) heading_whole, (unsigned int) heading_fraction);

	if( (body_length < 0) || ( (size_t)body_length >= sizeof(body) ) )
	{
		return 0U;
	}

	checksum = NMEA_CalculateChecksum(body); // Get checksum value

	/* Write NMEA 0183 Sentence*/
	sentence_length = snprintf(buffer, buffer_size, "$%s*%02X\r\n", body,(unsigned int) checksum);

	if ((sentence_length < 0) || ((size_t)sentence_length >= buffer_size))
	{
		return 0U;
	}

	return (size_t)sentence_length;
}
