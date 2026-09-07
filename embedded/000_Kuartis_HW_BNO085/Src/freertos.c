/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno085.h"
#include "heading.h"
#include "kalman_1d.h"
#include "nmea.h"
#include "usart.h"
#include "uart_tx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*DEBUG Variables*/
typedef struct
{
    /* Calibrated, unfiltered magnetometer data */
    float x_uT;
    float y_uT;
    float z_uT;

    /* Kalman-filtered magnetometer data */
    float filtered_x_uT;
    float filtered_y_uT;

    /* Heading comparison */
    float heading_deg;
    float filtered_heading_deg;

    /* Calibration status */
    uint8_t accuracy;
    uint8_t calibration_ok;

} BNO085_DebugView_t;


typedef struct
{
	float heading_deg; // Fİltered heading value
}HeadingMessage_t;


typedef enum
{
    SENSOR_STATE_STARTUP = 0U,
    SENSOR_STATE_WAIT_PRODUCT_ID,
    SENSOR_STATE_CALIBRATING,
    SENSOR_STATE_RUNNING,
    SENSOR_STATE_RECOVERY

} SensorState_t;


typedef enum
{
    SENSOR_ERROR_NONE = 0U,
    SENSOR_ERROR_INIT,
    SENSOR_ERROR_PRODUCT_ID_TIMEOUT,
    SENSOR_ERROR_COMMUNICATION,
    SENSOR_ERROR_CONFIGURATION,
    SENSOR_ERROR_DATA_TIMEOUT

} SensorError_t;


typedef enum
{
    COMM_ERROR_NONE = 0U,
    COMM_ERROR_QUEUE_FULL,
    COMM_ERROR_NMEA_FORMAT,
    COMM_ERROR_UART_BUSY,
    COMM_ERROR_UART

} CommunicationError_t;


typedef struct
{
    SensorState_t sensor_state;

    SensorError_t sensor_error;
    CommunicationError_t communication_error;

    uint32_t sensor_recovery_count;
    uint32_t queue_drop_count;
    uint32_t nmea_error_count;
    uint32_t uart_busy_count;
    uint32_t uart_error_count;

} SystemStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO_MAG_CAL_INTERVAL_US    20000U // 50 Hz for calibration
#define BNO_MAG_RUN_INTERVAL_US   100000U // 10 HZ for mag run

#define MAG_KALMAN_Q_UT2_PER_S    1.0f

#define MAG_KALMAN_X_R_UT2        0.433f
#define MAG_KALMAN_Y_R_UT2        0.393f

#define MAG_KALMAN_P0_UT2         1.0f


#define BNO_PRODUCT_ID_TIMEOUT_MS    1000U
#define BNO_DATA_TIMEOUT_MS           500U
#define BNO_RECOVERY_DELAY_MS        1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile BNO085_DebugView_t bno_debug_view = {0}; // It's for debug
static osMessageQueueId_t heading_queue_handle;

static volatile SystemStatus_t system_status =
{
    .sensor_state = SENSOR_STATE_STARTUP,
    .sensor_error = SENSOR_ERROR_NONE,
    .communication_error = COMM_ERROR_NONE
};

/* USER CODE END Variables */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommunicationTask */
osThreadId_t CommunicationTaskHandle;
const osThreadAttr_t CommunicationTask_attributes = {
  .name = "CommunicationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartCommunicationTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	(void)xTask;
	(void)pcTaskName;

	Error_Handler();
}

/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	Error_Handler();
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	heading_queue_handle = osMessageQueueNew(4U, sizeof(HeadingMessage_t), NULL);
	if(heading_queue_handle == NULL)
	{
		Error_Handler();
	}
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of CommunicationTask */
  CommunicationTaskHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
	BNO085_MagData_t mag_data = {0};

	Kalman1D_t mag_x_filter = {0};
	Kalman1D_t mag_y_filter = {0};

	uint32_t previous_filter_tick = 0U;
	uint8_t filter_time_valid = 0U; // First sample flag

	uint32_t state_start_tick = 0U;
	uint32_t last_data_tick = 0U;


	(void)argument;

	/*
	 * Initialize independent scalar Kalman filters	 * for magnetic X and Y components.
	 *
	 */
	if (Kalman1D_Init(&mag_x_filter, MAG_KALMAN_Q_UT2_PER_S, MAG_KALMAN_X_R_UT2,MAG_KALMAN_P0_UT2) == 0U)
	{
		Error_Handler();
	}

	if (Kalman1D_Init(&mag_y_filter,MAG_KALMAN_Q_UT2_PER_S,MAG_KALMAN_Y_R_UT2,MAG_KALMAN_P0_UT2) == 0U)
	{
		Error_Handler();
	}

	system_status.sensor_state = SENSOR_STATE_STARTUP; // Init state



	for (;;)
	{
		switch(system_status.sensor_state)
		{
		case SENSOR_STATE_STARTUP:
		{
			bno_debug_view.calibration_ok = 0U;
			filter_time_valid = 0U;

			if(BNO085_Init() == HAL_OK)
			{
				state_start_tick = HAL_GetTick();
				system_status.sensor_state = SENSOR_STATE_WAIT_PRODUCT_ID; // After succes initialization, swtich the state
			}
			else // If initialization is not succesful
			{
				system_status.sensor_error = SENSOR_ERROR_INIT;
				system_status.sensor_recovery_count++; // System recovery counter
				state_start_tick = HAL_GetTick();
				system_status.sensor_state = SENSOR_STATE_RECOVERY; // Go to recovery state after initialization error

			}
			break;
		}
		case SENSOR_STATE_WAIT_PRODUCT_ID:
		{
			BNO085_Process();

			if (BNO085_GetLastStatus() != HAL_OK)
			{
				system_status.sensor_error = SENSOR_ERROR_COMMUNICATION;
				system_status.sensor_recovery_count++;
				state_start_tick = HAL_GetTick();
				system_status.sensor_state = SENSOR_STATE_RECOVERY; // Go to recovery state if BNO085 state is not OK

				break;
			}
			if(BNO085_IsProductIDReceived() != 0U)
			{
				if(BNO085_EnableMagCalibration() != HAL_OK)
				{
					system_status.sensor_error = SENSOR_ERROR_CONFIGURATION; // Mag calibration is not started succesfully
					system_status.sensor_recovery_count++;
					state_start_tick = HAL_GetTick();

					system_status.sensor_state = SENSOR_STATE_RECOVERY;

					break;
				}
				if(BNO085_EnableMagnetometer(BNO_MAG_CAL_INTERVAL_US) != HAL_OK)
				{
					system_status.sensor_error = SENSOR_ERROR_CONFIGURATION;
					system_status.sensor_recovery_count++;
					state_start_tick = HAL_GetTick();
					system_status.sensor_state = SENSOR_STATE_RECOVERY;

					break;
				}
				last_data_tick = HAL_GetTick();
				system_status.sensor_state = SENSOR_STATE_CALIBRATING; // Mag is enable and mag calibration is started.
			}
			else if((HAL_GetTick() - state_start_tick) >= BNO_PRODUCT_ID_TIMEOUT_MS)
			{
				system_status.sensor_error = SENSOR_ERROR_PRODUCT_ID_TIMEOUT;
				system_status.sensor_recovery_count++;
				state_start_tick = HAL_GetTick();
				system_status.sensor_state = SENSOR_STATE_RECOVERY;
			}
			break;
		}

		case SENSOR_STATE_CALIBRATING:
		{
			BNO085_Process();

			if(BNO085_GetLastStatus() != HAL_OK)
			{
				system_status.sensor_error =  SENSOR_ERROR_COMMUNICATION;
				system_status.sensor_recovery_count++;
				state_start_tick = HAL_GetTick();
			    system_status.sensor_state = SENSOR_STATE_RECOVERY;

			    break;
			}
			if (BNO085_GetMagnetometer(&mag_data) != 0U)
			{
				float raw_heading_deg;

				last_data_tick = HAL_GetTick();

				/*Debug variables*/
				bno_debug_view.x_uT = mag_data.x_uT;
				bno_debug_view.y_uT = mag_data.y_uT;
				bno_debug_view.z_uT = mag_data.z_uT;
				bno_debug_view.accuracy = mag_data.accuracy;

				if(Heading_Calculate(mag_data.x_uT, mag_data.y_uT, &raw_heading_deg) != 0U)
				{
					bno_debug_view.heading_deg =  raw_heading_deg; // DEBUG Variable for raw heading.
				}

				if(mag_data.accuracy == 3U) // If calibration is succesfull and accuracy is high
				{
					if(BNO085_EnableMagnetometer(BNO_MAG_RUN_INTERVAL_US) != HAL_OK) // MAG config 10 HZ
					{
						system_status.sensor_error = SENSOR_ERROR_CONFIGURATION;
						system_status.sensor_recovery_count++;
						state_start_tick = HAL_GetTick();

						system_status.sensor_state = SENSOR_STATE_RECOVERY;

						break;
					}
					bno_debug_view.calibration_ok = 1U; // Calibration is done

					/* Reset Kalman Filter */
					Kalman1D_Reset(&mag_x_filter);
					Kalman1D_Reset(&mag_y_filter);

					filter_time_valid = 0U;
					last_data_tick = HAL_GetTick();

					system_status.sensor_state = SENSOR_STATE_RUNNING;
				}
			}
			else if((HAL_GetTick() - last_data_tick) >= BNO_DATA_TIMEOUT_MS)
			{
				system_status.sensor_error = SENSOR_ERROR_DATA_TIMEOUT;

				system_status.sensor_recovery_count++;

				state_start_tick = HAL_GetTick();

				system_status.sensor_state = SENSOR_STATE_RECOVERY;
			}
			break;
		}

		case SENSOR_STATE_RUNNING:
		{
			BNO085_Process();

			if(BNO085_GetLastStatus() != HAL_OK)
			{
				system_status.sensor_error = SENSOR_ERROR_COMMUNICATION;

				system_status.sensor_recovery_count++;

				state_start_tick = HAL_GetTick();

				system_status.sensor_state = SENSOR_STATE_RECOVERY;

				break;
			}
			if(BNO085_GetMagnetometer(&mag_data) != 0U)
			{
				float raw_heading_deg;

				/*Kalman Filter outputs*/
				float filtered_x_uT;
				float filtered_y_uT;
				float filtered_heading_deg;

				float dt_s = 0.0f; // Sample time

				uint32_t current_tick = HAL_GetTick();

				last_data_tick = current_tick;

				/*Debug variables*/
				bno_debug_view.x_uT = mag_data.x_uT;
				bno_debug_view.y_uT = mag_data.y_uT;
				bno_debug_view.z_uT = mag_data.z_uT;
				bno_debug_view.accuracy = mag_data.accuracy;

				if (Heading_Calculate(mag_data.x_uT, mag_data.y_uT, &raw_heading_deg) != 0U)
				{
					bno_debug_view.heading_deg = raw_heading_deg;
				}

				if(filter_time_valid != 0U)
				{
					dt_s = (float)(current_tick - previous_filter_tick ) * 0.001f;
				}
				else
				{
					filter_time_valid = 1U;
				}

				previous_filter_tick = current_tick;

				if ((Kalman1D_Update(&mag_x_filter, mag_data.x_uT, dt_s,&filtered_x_uT) != 0U)&& (Kalman1D_Update(&mag_y_filter, mag_data.y_uT, dt_s,&filtered_y_uT) != 0U))
				{
					/*DEBUG variables*/
					 bno_debug_view.filtered_x_uT =  filtered_x_uT;
					 bno_debug_view.filtered_y_uT =  filtered_y_uT;

					 if(Heading_Calculate(filtered_x_uT, filtered_y_uT, &filtered_heading_deg) != 0U)
					 {
						 HeadingMessage_t heading_message;
						 bno_debug_view.filtered_heading_deg = filtered_heading_deg;

						 heading_message.heading_deg = filtered_heading_deg; // Read filtered heading for queue

						 if(osMessageQueuePut(heading_queue_handle, &heading_message, 0U, 0U) != osOK)
						 {
							 system_status.communication_error = COMM_ERROR_QUEUE_FULL;

							 system_status.queue_drop_count++;
						 }
					 }
				}
			}
			else if((HAL_GetTick() - last_data_tick) >= BNO_DATA_TIMEOUT_MS)
			{
				system_status.sensor_error = SENSOR_ERROR_DATA_TIMEOUT;
				system_status.sensor_recovery_count++;

				state_start_tick = HAL_GetTick();

				system_status.sensor_state = SENSOR_STATE_RECOVERY;
			}
			break;
		}

		case SENSOR_STATE_RECOVERY:
		{
			if( (HAL_GetTick() - state_start_tick) >= BNO_RECOVERY_DELAY_MS)
			{
				filter_time_valid = 0U;

				system_status.sensor_state = SENSOR_STATE_STARTUP;
			}
			break;
		}

		default:
		{
			system_status.sensor_error = SENSOR_ERROR_CONFIGURATION;
			system_status.sensor_recovery_count++;
			state_start_tick = HAL_GetTick();

			system_status.sensor_state = SENSOR_STATE_RECOVERY;

			break;
		}

	  }

	   osDelay(1U);
	}
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the CommunicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
    /* USER CODE BEGIN StartCommunicationTask */

    HeadingMessage_t heading_message;
    char nmea_sentence[NMEA_HDM_BUFFER_SIZE];

    (void)argument;

    if (UART_TX_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        if (osMessageQueueGet(heading_queue_handle,&heading_message, NULL,osWaitForever) == osOK)
        {
            size_t nmea_length;
            HAL_StatusTypeDef tx_status;

            nmea_length =  NMEA_FormatHDM(heading_message.heading_deg,nmea_sentence,sizeof(nmea_sentence));

            if (nmea_length == 0U)
            {
                system_status.communication_error = COMM_ERROR_NMEA_FORMAT;

                system_status.nmea_error_count++;

                continue;
            }

            tx_status =  UART_TX_Write((const uint8_t *)nmea_sentence,  (uint16_t)nmea_length);

            if (tx_status == HAL_BUSY)
            {
                system_status.communication_error = COMM_ERROR_UART_BUSY;

                system_status.uart_busy_count++;
            }
            else if (tx_status != HAL_OK)
            {
                system_status.communication_error = COMM_ERROR_UART;

                system_status.uart_error_count++;
            }
        }
    }

    /* USER CODE END StartCommunicationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

