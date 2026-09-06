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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*DEBUG Variables*/
typedef struct
{
    float x_uT;
    float y_uT;
    float z_uT;

    float heading_deg;

    float filtered_x_uT;
    float filtered_y_uT;
    float filtered_heading_deg;

    float kalman_x_gain;
    float kalman_x_innovation;
    float kalman_x_covariance;

    float kalman_y_gain;
    float kalman_y_innovation;
    float kalman_y_covariance;

    float noise_mean_x_uT;
    float noise_mean_y_uT;

    float noise_variance_x_uT2;
    float noise_variance_y_uT2;

    uint8_t accuracy;
    uint8_t calibration_ok;
    uint8_t kalman_active;

    uint8_t noise_test_state;

    uint32_t sample_count;
    uint32_t noise_sample_count;

} BNO085_DebugView_t;

/*Welford ALgorithm*/
typedef struct
{
    uint32_t count;
    float mean;
    float m2;

} RunningStats_t;


typedef enum
{
    MAG_NOISE_TEST_IDLE = 0,
    MAG_NOISE_TEST_SETTLING,
    MAG_NOISE_TEST_COLLECTING,
    MAG_NOISE_TEST_COMPLETE

} MagNoiseTestState_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO_MAG_CAL_INTERVAL_US    20000U // 50 Hz for calibration
#define BNO_MAG_RUN_INTERVAL_US   100000U // 10 HZ for mag run

#define MAG_KALMAN_Q_UT2_PER_S    1.0f

#define MAG_KALMAN_X_R_UT2        0.433f
#define MAG_KALMAN_Y_R_UT2        0.393f

#define MAG_KALMAN_P0_UT2         1.0f

/*Welford algorithm*/
#define MAG_NOISE_SETTLE_TIME_MS      3000U
#define MAG_NOISE_SAMPLE_COUNT        200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile BNO085_DebugView_t bno_debug_view = {0}; // It's for debug

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
/*Wrlford fuctions*/
static void RunningStats_Reset(RunningStats_t *stats);
static void RunningStats_Update(RunningStats_t *stats, float sample);
static float RunningStats_GetVariance(const RunningStats_t *stats);
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
	Error_Handler();
}

/*Welford functinos*/

static void RunningStats_Reset(RunningStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->count = 0U;
    stats->mean = 0.0f;
    stats->m2 = 0.0f;
}


static void RunningStats_Update(RunningStats_t *stats, float sample)
{
    float delta;
    float delta2;

    if (stats == NULL)
    {
        return;
    }

    stats->count++;

    delta = sample - stats->mean;

    stats->mean +=
        delta / (float)stats->count;

    delta2 = sample - stats->mean;

    stats->m2 +=
        delta * delta2;
}


static float RunningStats_GetVariance(const RunningStats_t *stats)
{
    if ((stats == NULL) || (stats->count < 2U))
    {
        return 0.0f;
    }

    /*
     * Sample variance.
     */
    return stats->m2 /
           (float)(stats->count - 1U);
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
	uint8_t calibration_request_sent = 0U;
	uint8_t magnetometer_configured = 0U;
	uint8_t normal_rate_configured = 0U;

	Kalman1D_t mag_x_filter = {0};
	Kalman1D_t mag_y_filter = {0};

	uint32_t previous_filter_tick = 0U;
	uint8_t filter_time_valid = 0U;

	/*Welford*/
	RunningStats_t mag_x_stats = {0};
	RunningStats_t mag_y_stats = {0};
	MagNoiseTestState_t noise_test_state =MAG_NOISE_TEST_IDLE;
	uint32_t noise_settle_start_tick = 0U;

	RunningStats_Reset(&mag_x_stats);
	RunningStats_Reset(&mag_y_stats);

	if (Kalman1D_Init(&mag_x_filter, MAG_KALMAN_Q_UT2_PER_S, MAG_KALMAN_X_R_UT2,MAG_KALMAN_P0_UT2) == 0U)
	{
		Error_Handler();
	}

	if (Kalman1D_Init(&mag_y_filter,MAG_KALMAN_Q_UT2_PER_S,MAG_KALMAN_Y_R_UT2,MAG_KALMAN_P0_UT2) == 0U)
	{
		Error_Handler();
	}
	(void)argument;

	if (BNO085_Init() != HAL_OK)
	{
		Error_Handler();
	}
  /* Infinite loop */
	for (;;)
	    {
	        BNO085_Process();

	        if ((BNO085_IsProductIDReceived() != 0U) &&
	            (calibration_request_sent == 0U))
	        {
	            if (BNO085_EnableMagCalibration() == HAL_OK)
	            {
	                calibration_request_sent = 1U;
	            }
	        }

	        if ((calibration_request_sent != 0U) &&
	            (magnetometer_configured == 0U))
	        {
	            if (BNO085_EnableMagnetometer(BNO_MAG_CAL_INTERVAL_US) == HAL_OK)
	            {
	                magnetometer_configured = 1U;
	            }
	        }
	        if (BNO085_GetMagnetometer(&mag_data) != 0U)
	        {
	        	float raw_heading_deg;
	        	float filtered_heading_deg;
	        	float filtered_x_uT;
	        	float filtered_y_uT;
	        	float dt_s = 0.0f;
	        	uint32_t current_tick;

	        	/*
	        	 * Raw calibrated magnetometer values.
	             */
	        	bno_debug_view.x_uT = mag_data.x_uT;
	        	bno_debug_view.y_uT = mag_data.y_uT;
	        	bno_debug_view.z_uT = mag_data.z_uT;

	        	bno_debug_view.accuracy = mag_data.accuracy;
	        	bno_debug_view.sample_count++;
	        	/*
	        	 * Raw heading:
	        	 * calibrated magnetometer -> atan2(Y, X)
	        	 */
	        	if (Heading_Calculate(mag_data.x_uT,mag_data.y_uT,&raw_heading_deg) != 0U)
	        	{
	        		 bno_debug_view.heading_deg = raw_heading_deg;
	        	}


	        	if ((mag_data.accuracy == 3U) && (normal_rate_configured == 0U))
	        	{
	        		bno_debug_view.calibration_ok = 1U;

	        		if (BNO085_EnableMagnetometer(BNO_MAG_RUN_INTERVAL_US) == HAL_OK)
	        		{
	        			normal_rate_configured = 1U;
	        			Kalman1D_Reset(&mag_x_filter);
	        			Kalman1D_Reset(&mag_y_filter);
	        			filter_time_valid = 0U;
	        			bno_debug_view.kalman_active = 1U;
	        			/*WLford*/
	        			 RunningStats_Reset(&mag_x_stats);
	        			 RunningStats_Reset(&mag_y_stats);
	        			 noise_settle_start_tick = HAL_GetTick();
	        			 noise_test_state =  MAG_NOISE_TEST_SETTLING;
	        			 bno_debug_view.noise_test_state =  (uint8_t)noise_test_state;
	        			 bno_debug_view.noise_sample_count = 0U;
	        		}
	        	}
	        	 /*
	        	     * Measurement-noise characterization.
	        	     *
	        	     * Uses RAW calibrated magnetometer values.
	        	     */
	        	  if (noise_test_state == MAG_NOISE_TEST_SETTLING)
	        	    {
	        	        if ((HAL_GetTick() - noise_settle_start_tick) >=
	        	            MAG_NOISE_SETTLE_TIME_MS)
	        	        {
	        	            RunningStats_Reset(&mag_x_stats);
	        	            RunningStats_Reset(&mag_y_stats);

	        	            noise_test_state =
	        	                MAG_NOISE_TEST_COLLECTING;

	        	            bno_debug_view.noise_test_state =
	        	                (uint8_t)noise_test_state;
	        	        }
	        	    }
	        	    else if (noise_test_state == MAG_NOISE_TEST_COLLECTING)
	        	    {
	        	        RunningStats_Update(&mag_x_stats,
	        	                            mag_data.x_uT);

	        	        RunningStats_Update(&mag_y_stats,
	        	                            mag_data.y_uT);

	        	        bno_debug_view.noise_sample_count =
	        	            mag_x_stats.count;

	        	        bno_debug_view.noise_mean_x_uT =
	        	            mag_x_stats.mean;

	        	        bno_debug_view.noise_mean_y_uT =
	        	            mag_y_stats.mean;

	        	        if (mag_x_stats.count >=
	        	            MAG_NOISE_SAMPLE_COUNT)
	        	        {
	        	            bno_debug_view.noise_variance_x_uT2 =
	        	                RunningStats_GetVariance(&mag_x_stats);

	        	            bno_debug_view.noise_variance_y_uT2 =
	        	                RunningStats_GetVariance(&mag_y_stats);

	        	            noise_test_state =
	        	                MAG_NOISE_TEST_COMPLETE;

	        	            bno_debug_view.noise_test_state =
	        	                (uint8_t)noise_test_state;
	        	        }
	        	    }


	        	/************************************************************/

	        	if (normal_rate_configured != 0U)
	        	{
	        		current_tick = HAL_GetTick();
	        		if (filter_time_valid != 0U)
	        		{
	        			 dt_s = (float)(current_tick - previous_filter_tick)  * 0.001f;
	        		}
	        		else
	        		{
	        			 dt_s = 0.0f;
	        			 filter_time_valid = 1U;
	        		}

	        		previous_filter_tick = current_tick;
	        		if ((Kalman1D_Update(&mag_x_filter,mag_data.x_uT,dt_s,&filtered_x_uT) != 0U) &&  (Kalman1D_Update(&mag_y_filter, mag_data.y_uT,dt_s, &filtered_y_uT) != 0U))
	        		{
	        			bno_debug_view.filtered_x_uT = filtered_x_uT;
	        			bno_debug_view.filtered_y_uT = filtered_y_uT;

	        			 bno_debug_view.kalman_x_gain = mag_x_filter.last_gain;
	        			 bno_debug_view.kalman_x_innovation = mag_x_filter.last_innovation;
	        			 bno_debug_view.kalman_x_covariance =  mag_x_filter.error_covariance;
	        			 bno_debug_view.kalman_y_gain = mag_y_filter.last_gain;
	        			 bno_debug_view.kalman_y_innovation =  mag_y_filter.last_innovation;
	        			 bno_debug_view.kalman_y_covariance = mag_y_filter.error_covariance;

	        			if (Heading_Calculate(filtered_x_uT,filtered_y_uT,&filtered_heading_deg) != 0U)
	        			{
	        				 bno_debug_view.filtered_heading_deg =filtered_heading_deg;
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommunicationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

