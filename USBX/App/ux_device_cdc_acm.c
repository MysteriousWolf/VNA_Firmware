/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
ULONG rx_actual_length = 0;
uint8_t UserRxBuffer[64] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  //UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* Save the instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */
VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input)
{
  /* Private Variables */
  ULONG tx_actual_length;
  ULONG tx_ideal_length;
  uint8_t message[QUEUE_STACK_SIZE] = {};
  while(1)
  {
    // Read the message from the queue
    tx_actual_length = receive_data_from_queue(&out_queue, message, &tx_ideal_length);
    ux_device_class_cdc_acm_write(cdc_acm, message, tx_actual_length, &tx_actual_length);

    // Send the message to the USB
    //ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(message), sizeof(message), &tx_actual_length);
    //tx_thread_sleep(100);
  }
}

VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
  /* Private Variables */
  rx_actual_length = 0;
  /* Infinite Loop */
  while(1)
  {
    if(cdc_acm != UX_NULL)
    {
      ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBuffer, 64, &rx_actual_length);
      vna_process_command(UserRxBuffer, rx_actual_length);
    }
  }
}
/* USER CODE END 1 */
