/*
 * bno085.h
 *
 *  Created on: 5 Eyl 2026
 *      Author: furkan
 */

#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "main.h"
typedef struct
{
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;

	float x_uT;
	float y_uT;
	float z_uT;

	uint8_t accuracy;
}BNO085_MagData_t;


HAL_StatusTypeDef BNO085_Init(void);

void BNO085_Process(void);

HAL_StatusTypeDef BNO085_EnableMagnetometer(uint32_t interval_us);

HAL_StatusTypeDef BNO085_EnableMagCalibration(void);




/* BNO085 GETTER FUNCTIONS */
void BNO085_NotifyInterrupt(void);

uint8_t BNO085_IsProductIDReceived(void);

HAL_StatusTypeDef BNO085_GetLastStatus(void);

uint8_t BNO085_GetResetCause(void);

uint8_t BNO085_IsSHTPErrorListReceived(void);

uint8_t BNO085_GetSHTPErrorCount(void);

uint8_t BNO085_GetSHTPError(uint8_t index);

uint8_t BNO085_GetMagnetometer(BNO085_MagData_t *data);



#endif /* INC_BNO085_H_ */
