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

#include "stm32f4xx_ll_fsmc.h"
#include "stm32f4xx_hal_nand.h"


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
	devMEM = 2,
	devI2C = 4,
	devRTC = 8,
	devSPI = 0x10,
	devNAND = 0x20,
	devQUE = 0x40,
	devSYS = 0x80
};

enum {
	cmdRestart = 0,
	cmdEpoch,
	cmdRead,
	cmdNext,
	cmdWrite,
	cmdErase,
	cmdCheck,
	cmdArea,
	cmdSave,
	cmdLog,
	cmdHelp
};

enum {
	logOff = 0,
	logOn,
	logDump,
	logNone
};


/**/
#pragma pack(push,1)
typedef struct {
	uint8_t cmd;
	uint8_t attr;
} s_qcmd;
#pragma pack(pop)
/**/

#pragma pack(push,1)
typedef struct
{
  uint8_t Maker_Id;
  uint8_t Device_Id;
  uint8_t Third_Id;
  uint8_t Fourth_Id;
  uint8_t Plane_Id;
} NAND_IDsTypeDef;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	uint32_t PageSize;      // NAND memory page (without spare area) size measured in bytes
	uint32_t SpareAreaSize; // NAND memory spare area size measured in bytes
	uint32_t BlockSize;     // NAND memory block size measured in number of pages
	uint32_t BlockNbr;      // NAND memory number of total blocks
	uint32_t PlaneNbr;      // NAND memory number of planes
	uint32_t PlaneSize;
} s_chipConf;
#pragma pack(pop)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */



#define MY_NAND_DEVICE 0x70000000L


//#define SET_SWV


#define MAX_CMDS       11//9//8
#define MAX_LEN_DATA  512//256
#define MAX_SCR_BUF  1024

#define MAX_UART_BUF     (MAX_LEN_DATA << 2)//1024
#define MAX_NAND_STATE   4
#define MAX_NAND_BUF   8192

#define LOOP_FOREVER() while(1) { HAL_Delay(1); }
#define HTONS(x) ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))


#ifdef SET_SWV
	#define MAX_TMP_SIZE 256
#endif

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
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOB
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOB
#define CLE_Pin GPIO_PIN_11
#define CLE_GPIO_Port GPIOD
#define ALE_Pin GPIO_PIN_12
#define ALE_GPIO_Port GPIOD
#define LED_ERR_Pin GPIO_PIN_13
#define LED_ERR_GPIO_Port GPIOD
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define LED_TIK_Pin GPIO_PIN_7
#define LED_TIK_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define NOE_Pin GPIO_PIN_4
#define NOE_GPIO_Port GPIOD
#define NWE_Pin GPIO_PIN_5
#define NWE_GPIO_Port GPIOD
#define NWAIT_Pin GPIO_PIN_6
#define NWAIT_GPIO_Port GPIOD
#define NCE2_Pin GPIO_PIN_7
#define NCE2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

extern uint16_t devError;
extern UART_HandleTypeDef *uartPort;
extern SPI_HandleTypeDef *ipsPort;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern bool spiRdy;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
