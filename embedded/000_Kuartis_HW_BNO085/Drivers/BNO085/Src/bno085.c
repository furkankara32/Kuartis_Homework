/*
 * bno085.c
 *
 *  Created on: 5 Eyl 2026
 *      Author: furkan
 */

#include "bno085.h"
#include "spi.h"



#define BNO_RX_BUFFER_SIZE     					300U
#define BNO_TX_BUFFER_SIZE      				64U
#define BNO_CHANNEL_COUNT        				6U

#define BNO_CHANNEL_SHTP_COMMAND             	0U // SHTP Channel
#define BNO_CHANNEL_CONTROL                  	2U // Sensor Hub Control Channel
#define BNO_CHANNEL_INPUT_REPORTS            	3U // Sensor report channel

#define BNO_SHTP_COMMAND_ERROR_LIST          	0x01U // SHTP Error List Request and Response ID

/*Product ID command and response*/
#define BNO_REPORT_PRODUCT_ID_REQUEST        	0xF9U
#define BNO_REPORT_PRODUCT_ID_RESPONSE       	0xF8U

/*Magnetometer commands*/
#define BNO_REPORT_SET_FEATURE_COMMAND        	0xFDU // Set feature command
#define BNO_REPORT_BASE_TIMESTAMP             	0xFBU // Base timestampp reference
#define BNO_REPORT_MAGNETIC_FIELD_CALIBRATED  	0x03U

/*Calibration commands*/
#define BNO_REPORT_COMMAND_REQUEST           	0xF2U
#define BNO_COMMAND_ME_CALIBRATION           	0x07U
#define BNO_COMMAND_INITIALIZE_UNSOLICITED   	0x84U

/* SH-2 command response */
#define BNO_REPORT_COMMAND_RESPONSE          	0xF1U

#define BNO_SHTP_ERROR_MAX_COUNT             	16U // SHTP include  16 type error



static volatile uint8_t bno_int_flag = 0U;

static uint8_t bno_header[4] = {0};
static uint8_t bno_rx_payload[BNO_RX_BUFFER_SIZE] = {0};
static uint8_t bno_tx_seq[BNO_CHANNEL_COUNT] = {0}; // SHTP Header sequence number.
static uint16_t bno_payload_len = 0U;
static uint8_t bno_product_id_received = 0U;

/* Related Errors */
static uint8_t bno_reset_cause = 0U; // Channel 2 prouct ID respond payload[1] value. İnclude reset cause value.
static uint8_t bno_shtp_error_request_sent = 0U; // Channel 2 product ID request flag.
static uint8_t bno_shtp_error_list_received = 0U; // Channel 0 SHTP layer Eror List response flag.
static uint8_t bno_shtp_error_count = 0U; // The number of errors in the error list obtained from Channel 0 SHTP error list response
static uint8_t bno_shtp_errors[BNO_SHTP_ERROR_MAX_COUNT] = {0}; // Channel 0 SHTP error list
static HAL_StatusTypeDef bno_last_status = HAL_OK;


/* Magnetometer static variables*/
static BNO085_MagData_t bno_mag_data = {0};
static volatile uint8_t bno_mag_data_ready = 0U;
static uint8_t bno_command_seq = 0U;  // SH-2 Command REquest payload sequence number
static uint8_t bno_init_complete_received = 0U;


/* ---------- PRIVATE FUNCTION PROTOTYPES ---------- */

static void BNO085_Reset(void);

static HAL_StatusTypeDef BNO085_WaitForStartupInt(uint32_t timeout_ms);

static HAL_StatusTypeDef BNO085_ReadPacket(void);

static HAL_StatusTypeDef BNO085_WaitForIntLow(uint32_t timeout_ms);

static HAL_StatusTypeDef BNO085_SendPacket(uint8_t channel,const uint8_t *payload,uint16_t payload_len);

static HAL_StatusTypeDef BNO085_RequestProductID(void);

static HAL_StatusTypeDef BNO085_DrainStartupPackets(void);

static HAL_StatusTypeDef BNO085_RequestSHTPErrorList(void);




/**
  * @brief  Read packet function
  * @param  void
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_ReadPacket(void)
{
	uint8_t dummy_header[4] = {0};
	uint16_t raw_length;
	uint16_t packet_len;
	HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET);

	/* ---------- SHTP HEADER ---------- */
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, dummy_header, bno_header, 4, 100);

	if(status != HAL_OK)
	{
		HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

		return status;
	}

	/* bno_header[0] == LSB Bits Of length, bno_header[1] MSB Bits Of length and 15 bit is continuation bit*/
	raw_length = ( (uint16_t)bno_header[0] | ( (uint16_t)bno_header[1] << 8) );

	// Data lenght can not be 0xFFFF it means Error
	if(raw_length == 0xFFFFU)
	{
		HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

		return HAL_ERROR;
	}


	/* Bits 14:0 = Real packet length */
	packet_len = raw_length & 0x7FFFU;

	if(packet_len == 0U)
	{
		/* NULL HEADER*/
		bno_payload_len = 0U;
		HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

		return HAL_OK;
	}

	if (packet_len < 4U)
	{
		/* HEADER CAN NOT BE LESS THAN 4 BYTES*/
		HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

		return HAL_ERROR;
	}

	bno_payload_len = packet_len - 4U;

	if(bno_payload_len > sizeof(bno_rx_payload) )
	{
		HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

		return HAL_ERROR;
	}

	/* ---------- PAYLOAD ----------*/
	if (bno_payload_len > 0U)
	{
		uint8_t dummy_payload[BNO_RX_BUFFER_SIZE] = {0};
		status = HAL_SPI_TransmitReceive(&hspi2, dummy_payload, bno_rx_payload,bno_payload_len, 100);
	}
	else
	{
		status = HAL_OK;
	}

	HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

	return status;
}



/**
  * @brief  Send Packet function
  *
 * 1. WAKE Pin -> LOW = STM32 wants to transmit data
 * 2. INT Pin -> LOW = BNO085 is ready for communication
 * 3. Handshake is done. CS->LOW = STM32 starts communication
 * 4. SHTP HEADER + PAYLOAD send is done CS-> HIGH
 * 5. WAKE Pin -> HIGH transmisson is done.
 * 6. Incremet the related channel's tx sequence.
 *
  * @param  void
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_SendPacket(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
	uint8_t tx_packet[BNO_TX_BUFFER_SIZE] = {0};
	uint8_t rx_discard[BNO_TX_BUFFER_SIZE] = {0}; // SPI garbage receive data.

	HAL_StatusTypeDef status;

	uint16_t total_len = payload_len + 4U;

	if(channel >= BNO_CHANNEL_COUNT)
	{
		return HAL_ERROR;
	}

	if(total_len > BNO_TX_BUFFER_SIZE)
	{
		return HAL_ERROR;
	}

	/* ---------- SHTP HEADER ---------- */
	/* SHTP HEADER = LENGTH LSB + LENGTH MSB + CHANNEL + SEQUENCE*/
	tx_packet[0] = (uint8_t)(total_len & 0xFFU); // Length LSB
	tx_packet[1] = (uint8_t)((total_len >> 8) & 0x7FU); // //Length MSB and continuation bit = 0
	tx_packet[2] = channel;
	tx_packet[3] = bno_tx_seq[channel];

	/* ---------- PAYLOAD ---------- */
	for(uint16_t i = 0U; i < payload_len; i++)
	{
		tx_packet[4U + i] = payload[i]; // Write the payload value after the header
	}

	HAL_GPIO_WritePin(BNO_P0_WAKE_GPIO_Port, BNO_P0_WAKE_Pin, GPIO_PIN_RESET);

	status = BNO085_WaitForIntLow(20U); // Wait for BNO085 is ready for transfer

	if(status != HAL_OK)
	{
		/* If BNO085 can not be Ready finish the transfer and return the error status*/

		HAL_GPIO_WritePin(BNO_P0_WAKE_GPIO_Port, BNO_P0_WAKE_Pin, GPIO_PIN_SET);

		return status;
	}

	bno_int_flag = 0U; // Clear flag  in the function. The purpose here is to prevent  it from being mistaken for an 'INT' assertion in the main loop

	/* ---------- SPI TRANSACTION ---------- */

	HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET); // Start transfer

	status = HAL_SPI_TransmitReceive(&hspi2, tx_packet, rx_discard, total_len, 100); // Transmit the SHTP packet

	HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET); // Transfer is done
	HAL_GPIO_WritePin(BNO_P0_WAKE_GPIO_Port, BNO_P0_WAKE_Pin, GPIO_PIN_SET); // Wake request is done

	if(status == HAL_OK)
	{
		// Transfer is done succesfully increment the related channel's tx sequence number.
		bno_tx_seq[channel]++;
	}

	return status;
}

/**
  * @brief  This function checks INT is low. If it is low it means SPI ready for transfer
  * @param  timeout_ms = Max timeout value
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_WaitForIntLow(uint32_t timeout_ms)
{
	uint32_t start_tick = HAL_GetTick();

	while(HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET)
	{
		if( (HAL_GetTick() - start_tick) >= timeout_ms)
		{
			return HAL_TIMEOUT;
		}

	}

	return HAL_OK;
}

/**
  * @brief  This function reads the startup packets(SHTP Advertisement-> Channel 0)
  * @param  void
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_DrainStartupPackets(void)
{
    HAL_StatusTypeDef status;

    uint32_t start_tick;
    uint32_t quiet_tick = 0U;

    status = BNO085_WaitForStartupInt(200U);

    if (status != HAL_OK)
    {
        return status;
    }

    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < 500U)
    {
        if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) == GPIO_PIN_RESET)
        {
            bno_int_flag = 0U;

            status = BNO085_ReadPacket();

            if (status != HAL_OK)
            {
                return status;
            }

            /* Check unsolicited Initialize Response */
            if ((bno_header[2] == BNO_CHANNEL_CONTROL) &&
                (bno_payload_len >= 6U) &&
                (bno_rx_payload[0] == BNO_REPORT_COMMAND_RESPONSE) &&
                (bno_rx_payload[2] == BNO_COMMAND_INITIALIZE_UNSOLICITED) &&
                (bno_rx_payload[5] == 0U))
            {
                bno_init_complete_received = 1U;
            }

            quiet_tick = HAL_GetTick();
        }
        else
        {

            /*
             * Initialization is complete and no new packet * has arrived for 50 ms.
             */
            if ((bno_init_complete_received != 0U) &&  ((HAL_GetTick() - quiet_tick) >= 50U))
            {
                return HAL_OK;
            }
        }
    }

    return HAL_TIMEOUT;
}

/**
  * @brief  This function reset the BNO085
  * @param  void
  * @retval void
  */
static void BNO085_Reset(void)
{
	bno_int_flag = 0U;
	bno_product_id_received = 0U;
	bno_payload_len = 0U;
	bno_reset_cause = 0U;
	bno_shtp_error_request_sent = 0U;
	bno_shtp_error_list_received = 0U;
	bno_shtp_error_count = 0U;

	bno_command_seq = 0U;

	bno_mag_data_ready = 0U;

	bno_mag_data.x_uT = 0.0f;
	bno_mag_data.y_uT = 0.0f;
	bno_mag_data.z_uT = 0.0f;
	bno_mag_data.accuracy = 0U;

	bno_init_complete_received = 0U;
	/* Clear Error List*/
	for (uint8_t i = 0U; i < BNO_SHTP_ERROR_MAX_COUNT; i++)
	{
	    bno_shtp_errors[i] = 0U;
	}
	/* Clear all tx sequence */
	for (uint8_t i = 0U; i < BNO_CHANNEL_COUNT; i++)
	{
		bno_tx_seq[i] = 0U;
	}

	/* SPI mode Selection */
	HAL_GPIO_WritePin(BNO_P0_WAKE_GPIO_Port, BNO_P0_WAKE_Pin, GPIO_PIN_SET); // PO High , P1-> 3.3V

	/* CS idle HIGH */
	HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

	/* BN085 Hardware RESET */
	HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);

	HAL_Delay(5);

	HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

	bno_int_flag = 0U;

}

/**
  * @brief  This function waits Int pin low and int flag is 1 in the startup.
  * @param  timeout_ms = Max timeout value.
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_WaitForStartupInt(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

	while ((bno_int_flag == 0U) && (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) == GPIO_PIN_SET))
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) != GPIO_PIN_RESET)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
  * @brief  This function request the product ID
  * @param  void
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_RequestProductID(void)
{
	uint8_t payload[2];

	payload[0] = BNO_REPORT_PRODUCT_ID_REQUEST; //Product ID request command
	payload[1] = 0x00U;

	/* Channel 2 = SH2 Control Channel , Channel 2 0xF9 = Product ID request*/
	return BNO085_SendPacket(BNO_CHANNEL_CONTROL, payload, sizeof(payload) ) ;

}

/**
  * @brief  This function request SHTP Error List
  * @param  void
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BNO085_RequestSHTPErrorList(void)
{
	uint8_t payload[1];

	payload[0] = BNO_SHTP_COMMAND_ERROR_LIST;

	return BNO085_SendPacket(BNO_CHANNEL_SHTP_COMMAND, payload, sizeof(payload) );
}

/**
  * @brief  Returns the most recent HAL status recorded by the BNO085 driver.
  * @param  void
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BNO085_GetLastStatus(void)
{
    return bno_last_status;
}

/**
  * @brief  Notifies the BNO085 driver that a sensor interrupt has occurred.
  * @param  void
  * @retval void
  */
void BNO085_NotifyInterrupt(void)
{
    bno_int_flag = 1U;
}

/**
  * @brief  Initializes the BNO085 sensor , Resets the sensor, drains startup packets, sends a Product ID request to verify communication with the device.
  *
  * @param  void
  * @retval void
  */
HAL_StatusTypeDef BNO085_Init(void)
{
	bno_last_status = HAL_OK;

    BNO085_Reset();

    bno_last_status = BNO085_DrainStartupPackets();

    if (bno_last_status != HAL_OK)
    {
        return bno_last_status;
    }

    bno_last_status = BNO085_RequestProductID();

    return bno_last_status;
}

/**
 * @brief Checks whether a valid BNO085 Product ID response has been received.
 *
 * @return 1 if the Product ID response has been received, otherwise 0.
 */
uint8_t BNO085_IsProductIDReceived(void)
{
    return bno_product_id_received;
}

/**
 * @brief Returns the reset cause reported by the BNO085.
 *
 * @return BNO085 reset cause value obtained from the Product ID response.
 */
uint8_t BNO085_GetResetCause(void)
{
    return bno_reset_cause;
}

/**
 * @brief Checks whether an SHTP error list response has been received.
 *
 * @return 1 if the SHTP error list has been received, otherwise 0.
 */
uint8_t BNO085_IsSHTPErrorListReceived(void)
{
    return bno_shtp_error_list_received;
}

/**
 * @brief Returns the number of SHTP errors currently stored by the driver.
 *
 * @return Number of stored SHTP error entries.
 */
uint8_t BNO085_GetSHTPErrorCount(void)
{
    return bno_shtp_error_count;
}

/**
 * @brief Returns an SHTP error entry at the specified index.
 *
 * @param index Index of the requested SHTP error entry.
 *
 * @return SHTP error value at the requested index, or 0xFF if the index
 *         is outside the valid range.
 */
uint8_t BNO085_GetSHTPError(uint8_t index)
{
    if (index >= bno_shtp_error_count)
    {
        return 0xFFU;
    }

    return bno_shtp_errors[index];
}

/**
 * @brief Processes pending BNO085 packets when sensor data is available.
 *
 */
void BNO085_Process(void)
{

    if((bno_int_flag != 0U) || (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) == GPIO_PIN_RESET))
    {
    		if( (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) != GPIO_PIN_RESET) )
    		{
    			bno_int_flag = 0U;
    			return;
    		}

    		bno_int_flag = 0U;

    		bno_last_status = BNO085_ReadPacket();

    		if (bno_last_status != HAL_OK)
    		{
    			 return;
    		}

    		/* ---------- PRODUCT ID RESPONSE ---------- */
    		 if ( (bno_header[2] == BNO_CHANNEL_CONTROL) && (bno_payload_len >= 2U) && (bno_rx_payload[0] == BNO_REPORT_PRODUCT_ID_RESPONSE) )
    		 {
    			 bno_product_id_received = 1U;

    			 bno_reset_cause = bno_rx_payload[1];
    		 }

    		 /* ---------- SHTP ERROR LIST RESPONSE ---------- */

    		 if ((bno_header[2] == BNO_CHANNEL_SHTP_COMMAND) &&  (bno_payload_len >= 1U) && (bno_rx_payload[0] == BNO_SHTP_COMMAND_ERROR_LIST))
    		 {
    			  bno_shtp_error_count = 0U;

    			  for (uint16_t i = 1U; (i < bno_payload_len) && (bno_shtp_error_count < BNO_SHTP_ERROR_MAX_COUNT); i++)
    			  {
    				  bno_shtp_errors[bno_shtp_error_count] = bno_rx_payload[i];

    				  bno_shtp_error_count++;
    			  }

    			  bno_shtp_error_list_received = 1U;
    		 }

    		 /********** Magnetometer Report Check **********/
    		 if( (bno_header[2] == BNO_CHANNEL_INPUT_REPORTS) && (bno_payload_len >= 15U) && (bno_rx_payload[0] == BNO_REPORT_BASE_TIMESTAMP) && (bno_rx_payload[5] == BNO_REPORT_MAGNETIC_FIELD_CALIBRATED) )
    		 {
    			 int16_t x_raw;
    			 int16_t y_raw;
    			 int16_t z_raw;
    			 bno_mag_data.accuracy = bno_rx_payload[7] & 0x03U; //Read 2 bit accurac
    			 x_raw = (int16_t)((uint16_t)bno_rx_payload[9] | ((uint16_t)bno_rx_payload[10] << 8)); // X axis LSB and MSB
    			 y_raw = (int16_t)((uint16_t)bno_rx_payload[11] | ((uint16_t)bno_rx_payload[12] << 8)); // Y axis LSB and MSB
    			 z_raw = (int16_t)((uint16_t)bno_rx_payload[13] | ((uint16_t)bno_rx_payload[14] << 8)); // Z axis LSB and MSB

    			 /* BNO085 sends calibrated magnetic field values in Q4 fixed-point format. */
    			 bno_mag_data.x_uT = (float)x_raw / 16.0f; // Calibrated x axis data
    			 bno_mag_data.y_uT = (float)y_raw / 16.0f; // Calibrated y axis data
    			 bno_mag_data.z_uT = (float)z_raw / 16.0f; // Calibrated z axis data

    			 bno_mag_data_ready = 1U;
    		 }


    		 /* ---------- REQUEST SHTP ERROR LIST ONCE  ---------- */

    		 if ((bno_product_id_received != 0U) && (bno_shtp_error_request_sent == 0U) && (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port,BNO_INT_Pin) == GPIO_PIN_SET))
    		 {
    			 bno_last_status = BNO085_RequestSHTPErrorList();

    			 if (bno_last_status == HAL_OK)
    			 {
    				 bno_shtp_error_request_sent = 1U;
    			 }
    		 }

    }
}

/**
  * @brief  Enable and config magnetometer
  * @param  uint32_t interval_us = Report interval
  * @retval HAL status
  */
HAL_StatusTypeDef BNO085_EnableMagnetometer(uint32_t interval_us)
{
	uint8_t payload[17] = {0};

	payload[0] = BNO_REPORT_SET_FEATURE_COMMAND; // Set feature command
	payload[1] = BNO_REPORT_MAGNETIC_FIELD_CALIBRATED; // Feature report ID

	payload[2] = 0U; // Feature flag

	/* Change sensitivity payload[3-4]*/
	payload[3] = 0U;
	payload[4] = 0U;

	/*Report interval ---> It should be little endian*/
	payload[5] = (uint8_t)(interval_us);
	payload[6] = (uint8_t)(interval_us >> 8);
    payload[7] = (uint8_t)(interval_us >> 16);
	payload[8] = (uint8_t)(interval_us >> 24);

	/* Batch interval = 0 */
	payload[9]  = 0U;
	payload[10] = 0U;
	payload[11] = 0U;
	payload[12] = 0U;

    /* Sensor specific configuration = 0 */
    payload[13] = 0U;
    payload[14] = 0U;
    payload[15] = 0U;
    payload[16] = 0U;

    return BNO085_SendPacket(BNO_CHANNEL_CONTROL, payload, sizeof(payload));
}

/**
  * @brief  This function starts the magnetometer calibration
  * @param  void
  * @retval HAL status
  */
HAL_StatusTypeDef BNO085_EnableMagCalibration(void)
{
	uint8_t payload[12] = {0};

	payload[0] = BNO_REPORT_COMMAND_REQUEST;  // 0XF2 COmmand Request
	payload[1] = bno_command_seq; // Command sequence number
	payload[2] = BNO_COMMAND_ME_CALIBRATION; // Me calibration command

	payload[3] = 0U; // Accelerometer calibration disabled
	payload[4] = 0U; // Gyroscope calibration disabled
	payload[5] = 1U; // Magnetometer calibration enabled

	payload[6] = 0U; // Configure me subcommand
	payload[7] = 0U;

	/* P5-P8: Reserved */
	payload[8]  = 0U;
	payload[9]  = 0U;
	payload[10] = 0U;
	payload[11] = 0U;

	HAL_StatusTypeDef status = BNO085_SendPacket(BNO_CHANNEL_CONTROL, payload, sizeof(payload));

	if(status == HAL_OK)
	{
		bno_command_seq++;
	}

	return status;
}
/**
  * @brief  Get magnetometer values
  *
  * @param  NO085_MagData_t *data = Magnetometer data structure
  *
  * @retval  0 or 1
  */
uint8_t BNO085_GetMagnetometer(BNO085_MagData_t *data)
{
	if( (data == NULL) || (bno_mag_data_ready == 0U) )
	{
		return 0U;
	}

	*data = bno_mag_data;
	bno_mag_data_ready = 0U; // Clear after reading

	return 1U;
}




