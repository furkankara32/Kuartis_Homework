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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile HAL_StatusTypeDef bno_debug_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef bno_debug_last_status = HAL_ERROR;

volatile uint8_t bno_debug_product_id = 0U;
volatile uint8_t bno_debug_reset_cause = 0U;

volatile uint8_t bno_debug_error_list_received = 0U;
volatile uint8_t bno_debug_error_count = 0U;

volatile HAL_StatusTypeDef bno_debug_mag_enable_status = HAL_ERROR;
volatile float bno_debug_mag_x = 0.0f;
volatile float bno_debug_mag_y = 0.0f;
volatile float bno_debug_mag_z = 0.0f;
volatile uint8_t bno_debug_mag_accuracy = 0U;

volatile HAL_StatusTypeDef bno_debug_mag_cal_status = HAL_ERROR;
uint8_t calibration_enabled = 0U;
uint8_t magnetometer_enabled = 0U;

volatile HAL_StatusTypeDef bno_debug_dcd_send_status = HAL_ERROR;

volatile uint8_t bno_debug_dcd_response_received = 0U;
volatile uint8_t bno_debug_dcd_status = 0xFFU;

volatile uint32_t bno_debug_sensor_loop_count = 0U;

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
	uint8_t calibration_enabled = 0U;
	uint8_t magnetometer_enabled = 0U;

	bno_debug_init_status = BNO085_Init();


  /* Infinite loop */
  for(;;)
  {

	  bno_debug_sensor_loop_count++;

	  BNO085_Process();

	  bno_debug_product_id = BNO085_IsProductIDReceived();
	  if ((bno_debug_product_id != 0U) &&  (calibration_enabled == 0U))
	  {
		  bno_debug_mag_cal_status = BNO085_EnableMagCalibration();


		  if (bno_debug_mag_cal_status == HAL_OK)
		  {
			  calibration_enabled = 1U;
		  }
	  }
	  if ((calibration_enabled != 0U) && (magnetometer_enabled == 0U))
	  {
		  bno_debug_mag_enable_status = BNO085_EnableMagnetometer(20000U);

		  if (bno_debug_mag_enable_status == HAL_OK)
		  {
			  magnetometer_enabled = 1U;
		  }
	  }
	  if (BNO085_GetMagnetometer(&mag_data) != 0U)
	  {
		  bno_debug_mag_x = mag_data.x_uT;
		  bno_debug_mag_y = mag_data.y_uT;
		  bno_debug_mag_z = mag_data.z_uT;

		  bno_debug_mag_accuracy = mag_data.accuracy;
	  }

	  osDelay(1);
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

