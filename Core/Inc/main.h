/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vna.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim1;

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_USB_PCD_Init(void);
void MX_SPI4_Init(void);

/* USER CODE BEGIN EFP */
void MX_USB_PCD_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_RST_Pin GPIO_PIN_0
#define SPI2_RST_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_3
#define SPI2_CS_GPIO_Port GPIOA
#define FPGA_CDONE_Pin GPIO_PIN_8
#define FPGA_CDONE_GPIO_Port GPIOE
#define FPGA_CRESET_Pin GPIO_PIN_9
#define FPGA_CRESET_GPIO_Port GPIOE
#define FPGA_CS_Pin GPIO_PIN_10
#define FPGA_CS_GPIO_Port GPIOE
#define SPI4_NSS_Pin GPIO_PIN_11
#define SPI4_NSS_GPIO_Port GPIOE
#define SPI4_SCK_Pin GPIO_PIN_12
#define SPI4_SCK_GPIO_Port GPIOE
#define SPI4_MISO_Pin GPIO_PIN_13
#define SPI4_MISO_GPIO_Port GPIOE
#define SPI4_MOSI_Pin GPIO_PIN_14
#define SPI4_MOSI_GPIO_Port GPIOE
#define CONVERSION_DONE_Pin GPIO_PIN_15
#define CONVERSION_DONE_GPIO_Port GPIOE
#define CONVERSION_DONE_EXTI_IRQn EXTI15_IRQn
#define SPI2_SCK_Pin GPIO_PIN_10
#define SPI2_SCK_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOD
#define V_SENSE_USB_Pin GPIO_PIN_6
#define V_SENSE_USB_GPIO_Port GPIOC
#define V_SENSE_FTDI_Pin GPIO_PIN_7
#define V_SENSE_FTDI_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_SCK_Pin GPIO_PIN_10
#define SPI3_SCK_GPIO_Port GPIOC
#define SPI3_RST_Pin GPIO_PIN_0
#define SPI3_RST_GPIO_Port GPIOD
#define BTN_1_Pin GPIO_PIN_1
#define BTN_1_GPIO_Port GPIOD
#define BTN_2_Pin GPIO_PIN_2
#define BTN_2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
