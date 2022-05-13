/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <malloc.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum {
	devUART = 1,
	devMEM,
	devI2C,
	devRTC,
	devSPI,
	devNAND
};

#pragma pack(push,1)
typedef struct {
	uint8_t restart:1;
	uint8_t time_set:1;
	uint8_t time_show:1;
	uint8_t none:5;
} s_flags;
#pragma pack(pop)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define MAX_UART_BUF 1024
#define MAX_TMP_SIZE 256
#define LOOP_FOREVER() while(1) { HAL_Delay(1); }
#define HTONS(x) \
    ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


#define _1s 4


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IPS_RES_Pin GPIO_PIN_1
#define IPS_RES_GPIO_Port GPIOA
#define IPS_DC_Pin GPIO_PIN_2
#define IPS_DC_GPIO_Port GPIOA
#define IPS_BLK_Pin GPIO_PIN_3
#define IPS_BLK_GPIO_Port GPIOA
#define IPS_SCK_Pin GPIO_PIN_5
#define IPS_SCK_GPIO_Port GPIOA
#define IPS_MOSI_Pin GPIO_PIN_7
#define IPS_MOSI_GPIO_Port GPIOA
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOB
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOB
#define LED_ERR_Pin GPIO_PIN_13
#define LED_ERR_GPIO_Port GPIOD
#define LED_TIK_Pin GPIO_PIN_7
#define LED_TIK_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

extern uint32_t devError;
extern UART_HandleTypeDef *uartPort;
extern SPI_HandleTypeDef *ipsPort;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern bool spiRdy;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
