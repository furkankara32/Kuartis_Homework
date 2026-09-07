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
	uint32_t timestamp_ms; // Tİmestamp value

}HeadingMessage_t;

typedef struct
{
    uint32_t sample_count;

    uint32_t current_period_ms;
    uint32_t min_period_ms;
    uint32_t max_period_ms;

    float average_period_ms;
    float frequency_hz;
    float peak_to_peak_jitter_ms;

} CommunicationTimingDebug_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO_MAG_CAL_INTERVAL_US    20000U // 50 Hz for calibration
#define BNO_MAG_RUN_INTERVAL_US   100000U // 10 HZ for mag run

#define MAG_KALMAN_Q_UT2_PER_S    1.0f

#define MAG_KALMAN_X_R_UT2        0.433f
#define MAG_KALMAN_Y_R_UT2        0.393f

#define MAG_KALMAN_P0_UT2         1.0f

#define TIMING_WARMUP_SAMPLES    20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile BNO085_DebugView_t bno_debug_view = {0}; // It's for debug
static osMessageQueueId_t heading_queue_handle;

static volatile float comm_debug_heading_deg = 0.0f;
static volatile uint32_t comm_debug_timestamp_ms = 0U;

static char comm_debug_nmea[NMEA_HDM_BUFFER_SIZE] = {0};
static volatile uint32_t comm_debug_nmea_length = 0U;

static volatile uint32_t comm_debug_tx_count = 0U;
static volatile uint32_t comm_debug_uart_error_count = 0U;

static volatile CommunicationTimingDebug_t comm_timing_debug = {0};

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


	uint8_t calibration_request_sent = 0U;
	uint8_t magnetometer_configured = 0U;
	uint8_t normal_rate_configured = 0U;


	uint32_t previous_filter_tick = 0U;
	uint8_t filter_time_valid = 0U;

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



	if (BNO085_Init() != HAL_OK)
	{
		Error_Handler();
	}


  /* Infinite loop */
	for (;;)
	{
		BNO085_Process();

		/*
		 * Enable dynamic magnetometer calibration
		 */
		if ((BNO085_IsProductIDReceived() != 0U) && (calibration_request_sent == 0U))
		{

			if (BNO085_EnableMagCalibration() == HAL_OK)
			{
				calibration_request_sent = 1U;
			}
		}

		/*
		 * 50 Hz reports while calibrating
		 */
		if ((calibration_request_sent != 0U) && (magnetometer_configured == 0U))
		{

			if (BNO085_EnableMagnetometer(BNO_MAG_CAL_INTERVAL_US) == HAL_OK)
			{
				magnetometer_configured = 1U;
			}
		}


		if (BNO085_GetMagnetometer(&mag_data) != 0U)
		{

			float raw_heading_deg;

			/*
			 * Calibrated but unfiltered magnetic-field data.
			 */
			bno_debug_view.x_uT = mag_data.x_uT;
			bno_debug_view.y_uT = mag_data.y_uT;
			bno_debug_view.z_uT = mag_data.z_uT;

			bno_debug_view.accuracy = mag_data.accuracy;

			/*
			 * Raw heading:calibrated but unfiltered magnetometer
			 */

			if (Heading_Calculate(mag_data.x_uT, mag_data.y_uT,&raw_heading_deg) != 0U)
			{

				bno_debug_view.heading_deg = raw_heading_deg;
			}

			/*
			 * After high calibration accuracy start normal operation at 10Hz
			 */
			if ((mag_data.accuracy == 3U) && (normal_rate_configured == 0U))
			{
				bno_debug_view.calibration_ok = 1U;

				if (BNO085_EnableMagnetometer(BNO_MAG_RUN_INTERVAL_US)	== HAL_OK)
				{
					normal_rate_configured = 1U;

					Kalman1D_Reset(&mag_x_filter);
					Kalman1D_Reset(&mag_y_filter);

					filter_time_valid = 0U;
				}
			}

			/*
			 * Apply Kalman Fİlter
			 */
			if (normal_rate_configured != 0U)
	        {

				 float filtered_x_uT;
				 float filtered_y_uT;
				 float filtered_heading_deg;
				 float dt_s = 0.0f;

				 uint32_t current_tick = HAL_GetTick();

	        	 if (filter_time_valid != 0U)
	        	 {
	        		  dt_s = (float)(current_tick - previous_filter_tick)  * 0.001f;
	        	 }
	       		 else
	       		 {
	       			  filter_time_valid = 1U;
	       		 }

	       		 previous_filter_tick = current_tick;
	       		 if ((Kalman1D_Update(&mag_x_filter,mag_data.x_uT,dt_s,&filtered_x_uT) != 0U) &&  (Kalman1D_Update(&mag_y_filter, mag_data.y_uT,dt_s, &filtered_y_uT) != 0U))
	       		 {
	       			 bno_debug_view.filtered_x_uT = filtered_x_uT;
	       			 bno_debug_view.filtered_y_uT = filtered_y_uT;

	       			 if (Heading_Calculate(filtered_x_uT,filtered_y_uT,&filtered_heading_deg) != 0U)
	        		 {
	       				 HeadingMessage_t heading_message;

	        			 bno_debug_view.filtered_heading_deg = filtered_heading_deg;

	        			 heading_message.heading_deg = filtered_heading_deg;
	        			 heading_message.timestamp_ms = HAL_GetTick();

	        			 (void)osMessageQueuePut(heading_queue_handle, &heading_message, 0U, 0U);
	        		 }
	        	 }
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

	uint32_t previous_tx_tick = 0U;
	uint32_t period_sum_ms = 0U;
	uint32_t timing_warmup_count = 0U;
	uint8_t timing_valid = 0U;

	(void)argument;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(heading_queue_handle, &heading_message, NULL, osWaitForever) == osOK)
	  {
		  size_t nmea_length;

		  comm_debug_heading_deg = heading_message.heading_deg;
		  comm_debug_timestamp_ms = heading_message.timestamp_ms;

		  nmea_length = NMEA_FormatHDM(heading_message.heading_deg,comm_debug_nmea,sizeof(comm_debug_nmea));

		  comm_debug_nmea_length = (uint32_t)nmea_length;

		  if (nmea_length > 0U)
		  {
			  uint32_t current_tx_tick;
			  current_tx_tick = HAL_GetTick();

			  if (timing_valid != 0U)
			  {
			      uint32_t period_ms;

			      period_ms =
			          current_tx_tick -
			          previous_tx_tick;

			      /*
			       * Ignore startup/transitional samples.
			       */
			      if (timing_warmup_count < TIMING_WARMUP_SAMPLES)
			      {
			          timing_warmup_count++;
			      }
			      else
			      {
			          comm_timing_debug.current_period_ms =
			              period_ms;

			          if (comm_timing_debug.sample_count == 0U)
			          {
			              comm_timing_debug.min_period_ms =
			                  period_ms;

			              comm_timing_debug.max_period_ms =
			                  period_ms;
			          }
			          else
			          {
			              if (period_ms <
			                  comm_timing_debug.min_period_ms)
			              {
			                  comm_timing_debug.min_period_ms =
			                      period_ms;
			              }

			              if (period_ms >
			                  comm_timing_debug.max_period_ms)
			              {
			                  comm_timing_debug.max_period_ms =
			                      period_ms;
			              }
			          }

			          period_sum_ms +=
			              period_ms;

			          comm_timing_debug.sample_count++;

			          comm_timing_debug.average_period_ms =
			              (float)period_sum_ms /
			              (float)comm_timing_debug.sample_count;

			          comm_timing_debug.frequency_hz =
			              1000.0f /
			              comm_timing_debug.average_period_ms;

			          comm_timing_debug.peak_to_peak_jitter_ms =
			              (float)(comm_timing_debug.max_period_ms -
			                      comm_timing_debug.min_period_ms);
			      }
			  }
			  else
			  {
			      timing_valid = 1U;
			  }

			  previous_tx_tick =
			      current_tx_tick;
			  if (HAL_UART_Transmit(&huart3,(uint8_t *)comm_debug_nmea,(uint16_t)nmea_length,20U) == HAL_OK)
			  {
				  comm_debug_tx_count++;
			  }
			  else
			  {
				  comm_debug_uart_error_count++;
			  }
		  }
	  }

  }
  /* USER CODE END StartCommunicationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

