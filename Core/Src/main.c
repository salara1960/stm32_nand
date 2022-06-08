/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//
//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"
//

#include "st7789.h"

#ifdef SET_SMALL_FS
	#include "io_fs.h"
#endif

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
 RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

NAND_HandleTypeDef hnand1;

/* Definitions for defTask */
osThreadId_t defTaskHandle;
const osThreadAttr_t defTask_attributes = {
  .name = "defTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myQue */
osMessageQueueId_t myQueHandle;
const osMessageQueueAttr_t myQue_attributes = {
  .name = "myQue"
};
/* Definitions for binSem */
osSemaphoreId_t binSemHandle;
const osSemaphoreAttr_t binSem_attributes = {
  .name = "binSem"
};
/* USER CODE BEGIN PV */


//const char *version = "ver.0.2 05.05.2022";
//const char *version = "ver.0.3 06.05.2022";
//const char *version = "ver.0.4 (08.05.2022)";
//const char *version = "ver.0.5 (11.05.2022)";
//const char *version = "ver.0.6 (12.05.2022)";
//const char *version = "ver.0.6.1 (12.05.2022)";
//const char *version = "ver.0.6.2 (13.05.2022)";
//const char *version = "ver.0.7 (19.05.2022)";
//const char *version = "ver.0.8 (20.05.2022)";
//const char *version = "ver.0.9 (21.05.2022)";
//const char *version = "ver.1.0 (23.05.2022)";
//const char *version = "ver.1.0.1 (23.05.2022)";
//const char *version = "ver.1.1 (24.05.2022)";
//const char *version = "ver.1.2 (25.05.2022)";
//const char *version = "ver.1.2.1 (26.05.2022)";
//const char *version = "ver.1.2.2 (26.05.2022)";
//const char *version = "ver.1.2.3 (27.05.2022)";
//const char *version = "ver.1.3.1 (01.06.2022)";
//const char *version = "ver.1.3.3 (06.06.2022)";
//const char *version = "ver.1.4 (07.06.2022)";
//const char *version = "ver.1.4.1 (08.06.2022)";
const char *version = "ver.1.5 (08.06.2022)";



const char *eol = "\r\n";
const char *s_cmds[MAX_CMDS] = {
		"restart",
		"epoch:",
		"read:",
		"next",
		"write:",
		"erase:",
		"check:",
		"log:",
		"info",
		"help"};
uint16_t devError;
uint8_t cmd_flag = 0;
const char *str_cmds[MAX_CMDS] = {
	"Restart",
	"Epoch",
	"Read",
	"Next",
	"Write",
	"Erase",
	"Check",
	"Log",
	"Info",
	"Help"
};

volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t msCounter = 0;//period 250ms
static char txBuf[MAX_UART_BUF] = {0};
static char rxBuf[MAX_UART_BUF] = {0};
uint8_t rxByte = 0;
uint16_t ruk = 0;
bool uartRdy = true;
bool spiRdy = true;
bool setDate = false;
uint8_t tZone = 0;//2;
uint8_t dbg = logOn;
static uint32_t epoch = 1654720159;//1654694859;//1654694232;//1654614048;//1654613449;
//1654606136;//1654546759;//1654544747;
//1652998677;//1652445122;//1652361110;//1652296740;//1652042430;//1652037111;
//1653476796;//1653430034;//1653428168;//1653309745;//1653149140;//1653082240;//1653055492;
//1654096285;//1653681746;//1653652854;//1653602199;//1653563627;

SPI_HandleTypeDef *ipsPort = &hspi1;
TIM_HandleTypeDef *timePort = &htim2;
UART_HandleTypeDef *logPort = &huart3;
NAND_HandleTypeDef *nandPort = &hnand1;

const FontDef *fntKey = &Font_16x26;
const FontDef *tFont = &Font_11x18;
uint16_t back_color = BLACK;

uint32_t total_pages = 0;
uint32_t total_bytes = 0;
uint32_t devAdr = 0;
uint32_t nandAdr = 0;
uint16_t nandLen = 0;
uint32_t nandBlk = 0;
uint32_t nandPage = 0;
uint8_t nandByte = EMPTY;
uint32_t cb_nandCounter = 0;
HAL_NAND_StateTypeDef nandState = HAL_NAND_STATE_ERROR;
NAND_IDsTypeDef nandID = {0};
const uint8_t chipIDcode = 0xF1;
const char *chipID = "K9F1G08U0E";
s_chipConf chipConf = {0};
const char *nandAllState[MAX_NAND_STATE] = {
	"HAL_NAND_STATE_RESET",// = 0x00U,   NAND not yet initialized or disabled
	"HAL_NAND_STATE_READY",// = 0x01U,   NAND initialized and ready for use
	"HAL_NAND_STATE_BUSY", // = 0x02U,   NAND internal process is ongoing
	"HAL_NAND_STATE_ERROR" // = 0x03U    NAND error state
};
uint8_t *rdBuf = NULL;//rdBuf = (uint8_t *)calloc(1, chipConf.PageSize);
uint8_t *wrBuf = NULL;//wrBuf = (uint8_t *)calloc(1, chipConf.PageSize);


osStatus_t qStat;
char stx[MAX_UART_BUF];

#ifdef SET_SMALL_FS
	io_fs_file file;
	volatile int32_t fs_err;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
void defThread(void *argument);

/* USER CODE BEGIN PFP */

HAL_StatusTypeDef NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID);
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void errLedOn(bool on);
void set_Date(uint32_t usec);
uint32_t getSecRTC(RTC_HandleTypeDef *hrtc);
int sec2str(char *st);
uint8_t Report(const uint8_t addTime, const char *fmt, ...);
uint32_t nand_PageToBlock(const uint32_t page);
uint32_t nand_BlockToPage(const uint32_t blk);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------------------------------------------
#ifdef SET_SWV
/*
#include "stdio.h"
#define ITM_Port8(n)  (*((volatile unsigned char *)(0xE0000000 + 4*n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4*n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4*n)))
#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x1000000
struct __FILE { int handle; }
FILE __stdout;
FILE __stin;
int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return ch;
}
*/
/*
int __io_putchar(int ch)
{
	ITM_SendChar(ch);

	return ch;
}
*/
/**/
int _write(int file, char *buf, int len)
{
	for (int i = 0; i < len; i++) ITM_SendChar((*buf++));
	return len;
}
/**/
#endif

//-------------------------------------------------------------------------------------------
#if (USE_HAL_NAND_REGISTER_CALLBACKS == 1)
//pNAND_CallbackTypeDef pCallback
void HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand)
{
	if (hnand->Instance == FSMC_NAND_DEVICE) {
		cb_nandCounter++;
	}
}
//
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_FSMC_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  for (uint8_t i = 0; i < 4; i++) {
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(LED_TIK_GPIO_Port, LED_TIK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(LED_TIK_GPIO_Port, LED_TIK_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
  }

  // start timer2 + interrupt
  HAL_TIM_Base_Start_IT(timePort);

  HAL_UART_Receive_IT(logPort, &rxByte, 1);

  //set_Date(epoch);

  ST7789_Reset();
  ST7789_Init(back_color);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binSem */
  binSemHandle = osSemaphoreNew(1, 1, &binSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQue */
  myQueHandle = osMessageQueueNew (16, sizeof(s_qcmd), &myQue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defTask */
  defTaskHandle = osThreadNew(defThread, NULL, &defTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    LOOP_FOREVER();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 2499;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 21;
  sTime.Minutes = 52;
  sTime.Seconds = 12;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  set_Date(epoch);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  	  //  APB1 - 42MHz
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IPS_RES_Pin|IPS_DC_Pin|IPS_BLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_TIK_GPIO_Port, LED_TIK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IPS_RES_Pin IPS_DC_Pin IPS_BLK_Pin */
  GPIO_InitStruct.Pin = IPS_RES_Pin|IPS_DC_Pin|IPS_BLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ERR_Pin */
  GPIO_InitStruct.Pin = LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_ERR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_TIK_Pin */
  GPIO_InitStruct.Pin = LED_TIK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_TIK_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

	cb_nandCounter = 0;

  /* USER CODE END FSMC_Init 0 */

  FSMC_NAND_PCC_TimingTypeDef ComSpaceTiming = {0};
  FSMC_NAND_PCC_TimingTypeDef AttSpaceTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */
/*
  	hnand1.Config.PageSize = 2048;
    hnand1.Config.SpareAreaSize = 16;
    hnand1.Config.BlockSize = 131072;
    hnand1.Config.BlockNbr = 1024;
    hnand1.Config.PlaneNbr = 1;
    hnand1.Config.PlaneSize = 134217728;
    hnand1.Config.ExtraCommandEnable = DISABLE;
*/
  /* USER CODE END FSMC_Init 1 */

  /** Perform the NAND1 memory initialization sequence
  */
  hnand1.Instance = FSMC_NAND_DEVICE;
  /* hnand1.Init */
  hnand1.Init.NandBank = FSMC_NAND_BANK2;
  hnand1.Init.Waitfeature = FSMC_NAND_PCC_WAIT_FEATURE_ENABLE;
  hnand1.Init.MemoryDataWidth = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;
  hnand1.Init.EccComputation = FSMC_NAND_ECC_DISABLE;
  hnand1.Init.ECCPageSize = FSMC_NAND_ECC_PAGE_SIZE_256BYTE;
  hnand1.Init.TCLRSetupTime = 0;
  hnand1.Init.TARSetupTime = 0;
  /* hnand1.Config */
  hnand1.Config.PageSize = 2048;
  hnand1.Config.SpareAreaSize = 16;
  hnand1.Config.BlockSize = 64;//131072; (in pages)
  hnand1.Config.BlockNbr = 1024;
  hnand1.Config.PlaneNbr = 1;
  hnand1.Config.PlaneSize = 134217728;
  hnand1.Config.ExtraCommandEnable = DISABLE;
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 18;//252;
  ComSpaceTiming.WaitSetupTime = 12;//252;
  ComSpaceTiming.HoldSetupTime = 12;//252;
  ComSpaceTiming.HiZSetupTime = 6;//252;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 18;//252;
  AttSpaceTiming.WaitSetupTime = 12;//252;
  AttSpaceTiming.HoldSetupTime = 12;//252;
  AttSpaceTiming.HiZSetupTime = 6;//252;

  if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

    if (HAL_NAND_ECC_Disable(&hnand1) != HAL_OK) devError |= devNAND;

#if (USE_HAL_NAND_REGISTER_CALLBACKS == 1)
    if (HAL_NAND_RegisterCallback(&hnand1, HAL_NAND_IT_CB_ID, HAL_NAND_ITCallback) == HAL_ERROR) devError |= devNAND;
#endif

    if (NAND_Read_ID(&hnand1, &nandID) == HAL_OK) {//read ID information from chip

    	nandState = HAL_NAND_GetState(&hnand1);
/*
nand->cfg.page_size     = hwnand->Config.PageSize; // Page size (2048 for K9GAG08U0E )
nand->cfg.block_number  = hwnand->Config.BlockNbr; // Total Number of block in plane (1024 K9GAG08U0E )
nand->cfg.block_size    = hwnand->Config.BlockSize; // Block size (In page) (64 K9GAG08U0E )
nand->cfg.plane_number  = hwnand->Config.PlaneNbr;  // Number of plane (1 K9GAG08U0E )
nand->cfg.plane_size    = hwnand->Config.PlaneSize * nand->cfg.block_number; // Plane size (In Page)
*/
    	memcpy((uint8_t *)&chipConf, (uint8_t *)&hnand1.Config, sizeof(s_chipConf));

    	total_pages = chipConf.PlaneSize / chipConf.PageSize;
    	total_bytes = chipConf.PlaneSize;


    	if ((chipConf.PageSize > 0) && (chipConf.PageSize <= MAX_NAND_BUF)) {
    		rdBuf = (uint8_t *)calloc(1, chipConf.PageSize);
    		wrBuf = (uint8_t *)calloc(1, chipConf.PageSize);

    		if (!rdBuf || !wrBuf) devError |= devMEM;
    	} else {
    		devError |= devNAND;
    	}

    }


  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

//-------------------------------------------------------------------------------------------
uint32_t nand_PageToBlock(const uint32_t page)
{
    return (page * chipConf.PageSize) / chipConf.BlockSize;
}
//-------------------------------------------------------------------------------------------
uint32_t nand_BlockToPage(const uint32_t blk)
{
	return (blk * chipConf.BlockSize) / chipConf.PageSize;
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
uint32_t io_nand_get_page_size(void)
{
    return chipConf.PageSize;
}
uint32_t io_nand_get_block_number(void)
{
	return chipConf.BlockNbr;
}
uint32_t io_nand_get_block_size(void)
{
	return chipConf.BlockSize;
}
uint32_t io_nand_get_plane_number(void)
{
	return chipConf.PlaneNbr;
}
uint32_t io_nand_get_plane_size(void)
{
	return chipConf.PlaneSize;
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

HAL_StatusTypeDef NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID)
{

	if (hnand->State == HAL_NAND_STATE_BUSY) {

		return HAL_BUSY;

	} else if (hnand->State == HAL_NAND_STATE_READY) {

		__HAL_LOCK(hnand);
	    hnand->State = HAL_NAND_STATE_BUSY;

#if defined(FMC_Bank2_3)
	    if (hnand->Init.NandBank == FMC_NAND_BANK2) {
	    	devAdr = NAND_DEVICE1;
	    } else {
	    	devAdr = NAND_DEVICE2;
	    }
#else
	    devAdr = NAND_DEVICE;//MY_NAND_DEVICE;
#endif

	    /* Send Read ID command sequence */
	    *(__IO uint8_t *)((uint32_t)(devAdr | CMD_AREA))  = NAND_CMD_READID;
	    __DSB();
	    *(__IO uint8_t *)((uint32_t)(devAdr | ADDR_AREA)) = 0x00;
	    __DSB();

	    /* Read the electronic signature from NAND flash */
	    if (hnand->Init.MemoryDataWidth == FSMC_NAND_PCC_MEM_BUS_WIDTH_8) {
	    	__IO uint32_t data  = *(__IO uint32_t *)devAdr;
	    	__IO uint32_t data1 = *((__IO uint32_t *)devAdr + 4);

	    	pNAND_ID->Maker_Id   = ADDR_1ST_CYCLE(data);
	    	pNAND_ID->Device_Id  = ADDR_2ND_CYCLE(data);
	    	pNAND_ID->Third_Id   = ADDR_3RD_CYCLE(data);
	    	pNAND_ID->Fourth_Id  = ADDR_4TH_CYCLE(data);
	    	pNAND_ID->Plane_Id   = ADDR_1ST_CYCLE(data1);

	    	hnand->State = HAL_NAND_STATE_READY;
	    }

	    __HAL_UNLOCK(hnand);

	} else {
	    return HAL_ERROR;
	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef  NAND_Read_Page_8b(NAND_HandleTypeDef *hnand,
									   NAND_AddressTypeDef *pAddress,
									   uint8_t *pBuffer,
									   uint32_t size,
									   uint32_t offset)
{
uint8_t *buff = pBuffer;

	// Check the NAND controller state
	if (hnand->State == HAL_NAND_STATE_BUSY) {
		return HAL_BUSY;
	} else if (hnand->State == HAL_NAND_STATE_READY) {
		__HAL_LOCK(hnand);
	    hnand->State = HAL_NAND_STATE_BUSY;

	    uint32_t deviceaddress = devAdr;
	    uint32_t nandaddress = pAddress->Page;//ARRAY_ADDRESS(pAddress, hnand);

	    if (dbg > logOn)
	    	Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu%s",
	    		  __func__,
				  nandaddress,
				  pAddress->Page, pAddress->Plane, pAddress->Block, eol);

	    // Page(s) read loop
	    if (nandaddress < (hnand->Config.BlockSize * hnand->Config.BlockNbr)) {
	    	// Send read page command sequence
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
	    	__DSB();
	    	//
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = offset;//0x00U;
	    	__DSB();
	    	if ((hnand->Config.PageSize) > 512U) {
	    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = offset;//0x00U;
	    		__DSB();
	    	}
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	    	__DSB();
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	    	__DSB();
	    	if ((hnand->Config.BlockSize * hnand->Config.BlockNbr) > 65535U) {
	    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
	    		__DSB();
	    	}

	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA))  = NAND_CMD_AREA_TRUE1;
	    	__DSB();

	    	if (hnand->Config.ExtraCommandEnable == ENABLE) {
	    		uint32_t tickstart = HAL_GetTick();
	    		while (HAL_NAND_Read_Status(hnand) != NAND_READY) {// Read status until NAND is ready
	    			if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT) {
	    				hnand->State = HAL_NAND_STATE_ERROR;// Update the NAND controller state
	    				__HAL_UNLOCK(hnand);
	    				return HAL_TIMEOUT;
	    			}
	    		}

	    		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = ((uint8_t)0x00);
	    		__DSB();
	    	}

	    	for (uint32_t index = 0U; index < size; index++) {
	    		*buff++ = *(uint8_t *)deviceaddress;
	    	}

	    }

	    hnand->State = HAL_NAND_STATE_READY;
	    __HAL_UNLOCK(hnand);

	} else {
	    return HAL_ERROR;
	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef  NAND_Write_Page_8b(NAND_HandleTypeDef *hnand,
									  NAND_AddressTypeDef *pAddress,
									  uint8_t *pBuffer,
									  uint32_t size,
									  uint32_t offset)
{

	if (hnand->State == HAL_NAND_STATE_BUSY) {
		return HAL_BUSY;
	} else if (hnand->State == HAL_NAND_STATE_READY) {

	    __HAL_LOCK(hnand);
	    hnand->State = HAL_NAND_STATE_BUSY;

	    uint32_t deviceaddress = devAdr;
	    uint32_t nandaddress = pAddress->Page;//ARRAY_ADDRESS(pAddress, hnand);

	    if (dbg > logOn)
	    	Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu%s",
	    	      __func__,
	    		  nandaddress,
	    		  pAddress->Page, pAddress->Plane, pAddress->Block, eol);

	    if (nandaddress < (hnand->Config.BlockSize * hnand->Config.BlockNbr)) {
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
	    	__DSB();
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE0;
	    	__DSB();
	    	//
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = offset;//0x00U;
	    	__DSB();
	    	if (hnand->Config.PageSize > 512U) {
	    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = offset;//0x00U;
	    		__DSB();
	    	}
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	    	__DSB();
	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	    	__DSB();
	    	if ((hnand->Config.BlockSize * hnand->Config.BlockNbr) > 65535U) {
	    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
	    		__DSB();
	    	}
	    	//
	    	// Write data to memory
	    	for (uint32_t index = 0U; index < size; index++) {
	    		*(__IO uint8_t *)deviceaddress = *(pBuffer + index);
	    		__DSB();
	    	}

	    	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE_TRUE1;
	    	__DSB();

	    	uint32_t tickstart = HAL_GetTick();
	    	while (HAL_NAND_Read_Status(hnand) != NAND_READY) {// Read status until NAND is ready
	    		if ((HAL_GetTick() - tickstart) > NAND_WRITE_TIMEOUT) {
	    			hnand->State = HAL_NAND_STATE_ERROR;
	    			__HAL_UNLOCK(hnand);
	    			return HAL_TIMEOUT;
	    		}
	    	}

	    }

	    hnand->State = HAL_NAND_STATE_READY;
	    __HAL_UNLOCK(hnand);

	} else {
	    return HAL_ERROR;
	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef NAND_Erase_Block(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t prn)
{

	if (hnand->State == HAL_NAND_STATE_BUSY) {

		return HAL_BUSY;

	} else if (hnand->State == HAL_NAND_STATE_READY) {

		__HAL_LOCK(hnand);
		hnand->State = HAL_NAND_STATE_BUSY;

		uint32_t deviceaddress = devAdr;
		uint32_t nandaddress = pAddress->Page;//ARRAY_ADDRESS(pAddress, hnand);

		if (prn) {
			if (dbg > logOn)
				Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu%s",
			    	      __func__, nandaddress, pAddress->Page, pAddress->Plane, pAddress->Block, eol);
		}

		/* Send Erase block command sequence */
		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE0;
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
		__DSB();

		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE1;
		__DSB();

		hnand->State = HAL_NAND_STATE_READY;
		__HAL_UNLOCK(hnand);

	} else {

		return HAL_ERROR;

	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------
//      Check: page is empty ?
//
bool pageIsEmpty(uint32_t page)
{
bool ret = false;

	io_nand_read(page, rdBuf, chipConf.PageSize, 0)	;
	if (!(devError & devNAND)) {
		ret = true;
		for (uint32_t i = 0; i < chipConf.PageSize; i++) {
			if (*(uint8_t *)(rdBuf + i) != EMPTY) {
				ret = false;
				break;
			}
		}
	}

	return ret;
}
/*
//-------------------------------------------------------------------------------------------
HAL_StatusTypeDef nand_ReadPage(uint32_t page, uint8_t *buf)
{
	NAND_AddressTypeDef nans = {
		.Page = page,
		.Plane = 1,
		.Block = nand_PageToBlock(page)
	};

	return NAND_Read_Page_8b(nandPort, &nans, buf, chipConf.PageSize, 0);
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef nand_WritePage(uint32_t page, uint8_t *buf)
{
	NAND_AddressTypeDef nans = {
		.Page = page,
		.Plane = 1,
		.Block = nand_PageToBlock(page)
	};

	return NAND_Write_Page_8b(nandPort, &nans, buf, chipConf.PageSize, 0);
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef nand_EraseBlock(uint32_t block)
{
	NAND_AddressTypeDef nans = {
		.Page = nand_BlockToPage(block),
		.Plane = 1,
		.Block = block
	};

	return NAND_Erase_Block(nandPort, &nans, 0);
}
*/
#ifdef SET_SMALL_FS
int _write(int file, char *buf, int len)
{
	Report(0, "%.*s", len, buf);
	return len;
}
#endif
//
#ifdef SET_FS_TEST
static char *str_fsErr(int e)
{
	switch (e) {
		case LFS_ERR_OK://          = 0,    // No error
			return "OK";
		case LFS_ERR_IO://          = -5,   // Error during device operation
			return "ERR_IO";
		case LFS_ERR_CORRUPT://     = -84,  // Corrupted
			return "ERR_CORRUPT";
		case LFS_ERR_NOENT://       = -2,   // No directory entry
			return "ERR_NOENT";
		case LFS_ERR_EXIST://       = -17,  // Entry already exists
			return "ERR_EXIST";
		case LFS_ERR_NOTDIR://      = -20,  // Entry is not a dir
			return "ERR_NOTDIR";
		case LFS_ERR_ISDIR://       = -21,  // Entry is a dir
			return "ERR_ISDIR";
		case LFS_ERR_NOTEMPTY://    = -39,  // Dir is not empty
			return "ERR_NOTEMPTY";
		case LFS_ERR_BADF://        = -9,   // Bad file number
			return "ERR_BADF";
		case LFS_ERR_FBIG://        = -27,  // File too large
			return "ERR_FBIG";
		case LFS_ERR_INVAL://       = -22,  // Invalid parameter
			return "ERR_INVAL";
		case LFS_ERR_NOSPC://       = -28,  // No space left on device
			return "ERR_NOSPC";
		case LFS_ERR_NOMEM://       = -12,  // No more memory available
			return "ERR_NOMEM";
		case LFS_ERR_NOATTR://      = -61,  // No data/attr available
			return "ERR_NOATTR";
		case LFS_ERR_NAMETOOLONG:// = -36,  // File name too long
			return "ERR_NAMETOOLONG";
	}

	return "UNKNOWN";
}
#endif
//-----------------------------------------------------------------------------
uint32_t io_flash_adr_to_uint32(NAND_AddressTypeDef *adr)
{
	return  ((adr->Plane * chipConf.PlaneSize) + (adr->Block * chipConf.BlockSize) + adr->Page);
}
//-----------------------------------------------------------------------------
NAND_AddressTypeDef io_uint32_to_flash_adr(uint32_t addr)
{
NAND_AddressTypeDef a;

	a.Plane = addr / chipConf.PlaneSize;
	a.Block = (addr - a.Plane * chipConf.PlaneSize) / chipConf.BlockSize;
	a.Page  = addr - (a.Plane * chipConf.PlaneSize) - (a.Block * chipConf.BlockSize);

	return a;
}
//-----------------------------------------------------------------------------
uint32_t io_nand_read_8b (uint32_t addr, uint8_t *pBuffer, uint32_t size, uint32_t offset)
{
NAND_AddressTypeDef Address = io_uint32_to_flash_adr(addr);


    if (nandPort->State == HAL_NAND_STATE_BUSY) return HAL_BUSY;

    __HAL_LOCK(nandPort);
    nandPort->State = HAL_NAND_STATE_BUSY;

    uint32_t deviceaddress = devAdr;
    uint32_t nandaddress = ARRAY_ADDRESS(&Address, nandPort);

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
    __DSB();

    // Cards with page size <= 512 bytes
    if ((nandPort->Config.PageSize) <= 512U) {
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
        if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
			*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
			__DSB();
        }
    } else {// (hnand->Config.PageSize) > 512
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
    	}
    }

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA))  = NAND_CMD_AREA_TRUE1;
    __DSB();

    uint32_t tickstart = 0U;
    // Check if an extra command is needed for reading pages
    if (nandPort->Config.ExtraCommandEnable == ENABLE) {
        tickstart = HAL_GetTick();
        while (HAL_NAND_Read_Status(nandPort) != NAND_READY) {
            if((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT) {
            	nandPort->State = HAL_NAND_STATE_ERROR;// Update the NAND controller state
            	__HAL_UNLOCK(nandPort);
            	return HAL_TIMEOUT;
            }
        }

        // Go back to read mode
        *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
        __DSB();
    }

    // Get Data into Buffer
    uint8_t *buff = pBuffer;
    for (uint32_t index = 0; index < size; index++) *buff++ = *(uint8_t *)deviceaddress;

    nandPort->State = HAL_NAND_STATE_READY;
    __HAL_UNLOCK(nandPort);

    return HAL_OK;
}
//-----------------------------------------------------------------------------
uint32_t io_nand_write_8b(uint32_t addr, uint8_t *pBuffer, uint32_t size, uint32_t offset)
{
NAND_AddressTypeDef Address = io_uint32_to_flash_adr(addr);


    if (nandPort->State == HAL_NAND_STATE_BUSY) return HAL_BUSY;

    __HAL_LOCK(nandPort);
    nandPort->State = HAL_NAND_STATE_BUSY;

    uint32_t deviceaddress = devAdr;
    uint32_t nandaddress = ARRAY_ADDRESS(&Address, nandPort);

    /* Send write page command sequence */
    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
    __DSB();
    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE0;
    __DSB();

    /* Cards with page size <= 512 bytes */
    if (nandPort->Config.PageSize <= 512U) {
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
        }
    } else {/* (hnand->Config.PageSize) > 512 */
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(offset);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
        }
    }


    /* Write data to memory */
    uint8_t *buff = pBuffer;
    for (uint32_t index = 0; index < size; index++) {
    	*(__IO uint8_t *)deviceaddress = *buff++;
    	__DSB();
    }

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE_TRUE1;
    __DSB();

    /* Read status until NAND is ready */
    uint32_t tickstart;
    while(HAL_NAND_Read_Status(nandPort) != NAND_READY) {
        tickstart = HAL_GetTick();
        if ((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT) {
        	nandPort->State = HAL_NAND_STATE_ERROR;// Update the NAND controller state
        	__HAL_UNLOCK(nandPort);
            return HAL_TIMEOUT;
        }
    }

    nandPort->State = HAL_NAND_STATE_READY;
    __HAL_UNLOCK(nandPort);

    return HAL_OK;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef io_nand_erase_block(NAND_AddressTypeDef *pAddress)
{

	if (nandPort->State == HAL_NAND_STATE_BUSY) {

		return HAL_BUSY;

	} else if (nandPort->State == HAL_NAND_STATE_READY) {

		__HAL_LOCK(nandPort);
		nandPort->State = HAL_NAND_STATE_BUSY;

		uint32_t deviceaddress = devAdr;
		uint32_t nandaddress = ARRAY_ADDRESS(pAddress, nandPort);

		if (dbg > logOn)
			Report(1, "[%s] nand_adr:0x%X page:%lu block:%lu plane:%lu%s",
					__func__, nandaddress, pAddress->Page, pAddress->Block, pAddress->Plane, eol);

		/* Send Erase block command sequence */
		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE0;
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
		__DSB();
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
		__DSB();

		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE1;
		__DSB();

		nandPort->State = HAL_NAND_STATE_READY;
		__HAL_UNLOCK(nandPort);

	} else {

		return HAL_ERROR;

	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------
uint32_t io_nand_read(uint32_t addr, uint8_t *buffer, uint32_t size, uint32_t offset)
{

	if (io_nand_read_8b(addr, buffer, size, 0) != HAL_OK) devError |= devNAND;

    return 0;
}
//-----------------------------------------------------------------------------
uint32_t io_nand_write(uint32_t addr, uint8_t *buffer, uint32_t size, uint32_t offset)
{

	if (io_nand_write_8b(addr, buffer, size, offset) != HAL_OK) devError |= devNAND;

	return 0;
}
//-----------------------------------------------------------------------------
void io_nand_block_erase(uint32_t addr)
{
NAND_AddressTypeDef nans = io_uint32_to_flash_adr(addr);

	if (io_nand_erase_block(&nans) != HAL_OK) devError |= devNAND;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static const char *get_qStat(osStatus_t osStat)
{
	switch (osStat) {
		case 0:
			return "osOK";//                      =  0,         ///< Operation completed successfully.
		case -1:
			return "osError";//                   = -1,         ///< Unspecified RTOS error: run-time error but no other error message fits.
		case -2:
			return "osErrorTimeout";//            = -2,         ///< Operation not completed within the timeout period.
		case -3:
			return "osErrorResource";//           = -3,         ///< Resource not available.
		case -4:
			return "osErrorParameter";//          = -4,         ///< Parameter error.
		case -5:
			return "osErrorNoMemory";//           = -5,         ///< System is out of memory: it was impossible to allocate or reserve memory for the operation.
		case -6:
			return "osErrorISR";//                = -6,         ///< Not allowed in ISR context: the function cannot be called from interrupt service routines.
		case 0x7FFFFFFF:
			return "osStatusReserved";//          = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
	}

	return "UnknownError";
}
//-----------------------------------------------------------------------------
static const char *get_logName(uint8_t lg)
{
	switch (lg) {
		case logOff:
			return "logOff";
		case logOn:
			return "logOn";
		case logDump:
			return "logDump";
	}

	return "???";
}
//-----------------------------------------------------------------------------
//      Функция преобразует hex-строку в бинарное число типа uint32_t
//
uint32_t hex2bin(const char *buf, uint8_t len)
{
uint8_t i, j, jk, k;
uint8_t mas[8] = {0x30}, bt[2] = {0};
uint32_t dword, ret = 0;

    if (!len || !buf) return ret;
    if (len > 8) len = 8;
    k = 8 - len;
    memcpy(&mas[k], buf, len);

    k = j = 0;
    while (k < 4) {
        jk = j + 2;
        for (i = j; i < jk; i++) {
                 if ((mas[i] >= 0x30) && (mas[i] <= 0x39)) bt[i&1] = mas[i] - 0x30;
            else if ((mas[i] >= 0x61) && (mas[i] <= 0x66)) bt[i&1] = mas[i] - 0x57;//a,b,c,d,e,f
            else if ((mas[i] >= 0x41) && (mas[i] <= 0x46)) bt[i&1] = mas[i] - 0x37;//A,B,C,D,E,F
        }
        dword = (bt[0] << 4) | (bt[1] & 0xf);
        ret |= (dword << 8*(4 - k - 1));
        k++;
        j += 2;
    }

    return ret;
}
//-------------------------------------------------------------------------------------------
void errLedOn(bool on)
{
	if (on)
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);//LED ON
	else
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);//LED OFF
}
//------------------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint64_t get_msCounter()
{
	return msCounter;
}
//-----------------------------------------------------------------------------
void inc_msCounter()
{
	msCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_mstmr(uint64_t hs)
{
	return (get_msCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_mstmr(uint64_t hs)
{
	return (get_msCounter() >= hs ? true : false);
}
//-----------------------------------------------------------------------------------------
void set_Date(uint32_t usec)
{
struct tm ts;
time_t ep = (time_t)usec;

	gmtime_r(&ep, &ts);

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	sTime.Hours   = ts.tm_hour + tZone;
	sTime.Minutes = ts.tm_min;
	sTime.Seconds = ts.tm_sec;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
	else {
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
		else {
			setDate = true;
		}
	}
}
//----------------------------------------------------------------------------------------
uint32_t getSecRTC(RTC_HandleTypeDef *hrtc)
{
time_t ep = 0;
RTC_DateTypeDef sDate;

	if (HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN)) devError |= devRTC;
	else {
		RTC_TimeTypeDef sTime;
		if (HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN)) devError |= devRTC;
		else {
			struct tm ts;
			ts.tm_sec = sTime.Seconds;
			ts.tm_min = sTime.Minutes;
			ts.tm_hour = sTime.Hours;
			ts.tm_mday = sDate.Date;
			ts.tm_mon = sDate.Month - 1;
			ts.tm_year = sDate.Year;
			ts.tm_wday = sDate.WeekDay;
			ep = mktime(&ts);
		}
	}

	return ep;
}
//-----------------------------------------------------------------------------------------
int sec2str(char *st)
{
int ret = 0;

	if (!setDate) {
		uint32_t sec = get_secCounter();

		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;

		ret = sprintf(st, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) devError |= devRTC;
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) devError |= devRTC;
			else {
				ret = sprintf(st, "%02u.%02u %02u:%02u:%02u",
								sDate.Date, sDate.Month,
								sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}

	return ret;
}
//-------------------------------------------------------------------------------------------
uint8_t Report(const uint8_t addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;
char *buf = &txBuf[0];

	*buf = '\0';
	if (addTime) {
		dl = sec2str(buf);
		strcat(buf, " | ");
		dl += 3;
	}

	va_start(args, fmt);
	vsnprintf(buf + dl, len - dl, fmt, args);

	uartRdy = false;
	if (HAL_UART_Transmit_DMA(logPort, (uint8_t *)buf, strlen(buf)) != HAL_OK) devError |= devUART;
	while (!uartRdy) {} //HAL_Delay(1)

	va_end(args);

	return 0;
}
//-------------------------------------------------------------------------------------------
#ifdef SET_SMALL_FS


#endif
//------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {// logPort - log

		rxBuf[ruk++] = (char)rxByte;

		if (rxByte == 0x0a) {//end of line
			rxBuf[--ruk] = '\0';
			char *uk = NULL;
			bool check = false;
			cmd_flag = 0;
			s_qcmd qcmd = {0};
			if (strlen(rxBuf) >= 4) {
				int8_t idx = -1;
				for (int8_t i = 0; i < MAX_CMDS; i++) {
					if ((uk = strstr(rxBuf, s_cmds[i]))) {//const char *s_cmds ="restart"
														  //"epoch:"
														  //"read:0x4549ABBB:256"
						                                  //"next"
														  //"write:0x0:0x0:0xf0:256"//adr:[offset:]byte:len
														  //"erase:"
														  //"check:"
														  //"log:"
														  //"info"
						  	  	  	  	  	  	  	  	  //"help"
						idx = i;
						break;
					}
				}
				if ((uk == rxBuf) && (idx != -1)) {
					nandLen = MAX_LEN_DATA;//256;
					uk += strlen(s_cmds[idx]);
					char *uki = NULL, *uke = NULL, *ukb = NULL;
					switch (idx) {
						case cmdHelp:
						case cmdInfo:
						case cmdRestart:
							cmd_flag = 1;
						break;
						case cmdEpoch:
							if (strlen(uk) < 10) {
								qcmd.attr = 1;
							} else {
								uki = strchr(uk, ':');
								if (uki) {
									tZone = (uint8_t)atol(uki + 1);
									*uki = '\0';
								} else {
									tZone = 0;
								}
								epoch = (uint32_t)atol(uk);
							}
							cmd_flag = 1;
						break;
						case cmdRead://"read:0x4549ABBB:256";
							uki = strchr(uk, ':');
							if (uki) {
								nandLen = atol(uki + 1);
								*uki = '\0';
							}
							uki = strstr(uk, "0x");
							if (uki) {
								uki += 2;
								nandAdr = hex2bin(uki, strlen(uki));
							} else {
								nandAdr = atol(uk);
							}
							nandAdr += devAdr;
							check = true;
						break;
						case cmdNext://"next";
							if (nandAdr < devAdr) nandAdr = devAdr;
							nandAdr += nandLen;
							check = true;
						break;
						case cmdWrite://"write:'0x0:0x55:256'" //adr:byte:len
						{
							bool hex = false;
							uki = strstr(uk, "0x");
							if (uki) {
								if (uki == uk) {
									uki += 2;
									hex = true;
								}
							} else uki = uk;
							uke = strchr(uki, ':');//adr:byte
							if (uke) {
								ukb = uke + 1;
								char tmp[16];
								memset(tmp, 0, 16);
								memcpy(tmp, uki, uke - uki);
								if (hex) nandAdr = hex2bin(tmp, strlen(tmp));
								    else nandAdr = atol(tmp);
								uki = ukb;
								uke = strchr(uki, ':');//byte:len
								if (uke) {
									nandLen = atol(uke + 1);
									*uke = '\0';
								}
								if (strstr(ukb, "0x")) {
									ukb += 2;
									hex = true;
								} else hex = false;
								if (hex) nandByte = (uint8_t)hex2bin(ukb, strlen(ukb));
								    else nandByte = (uint8_t)atol(ukb);
								nandAdr += devAdr;
								check = true;
							}
						}
						break;
						case cmdErase://"erase:0" or "erase:all" //erase:block_number from 0..1023
							if (strstr(uk, "all")) {
								qcmd.attr = 1;
							} else {
								uint32_t blk = atol(uk);
								if (blk < chipConf.BlockNbr) nandBlk = blk;
							}
							cmd_flag = 1;
						break;
						case cmdCheck://"check:0" //check:page //(chipConf.BlockSize / chipConf.PageSize) * chipConf.BlockNbr
						{
							uint32_t page = atol(uk);
							if (page < total_pages) {//128MB / 2K = 65536 - pages
								nandPage = page;
								cmd_flag = 1;
							}
						}
						break;
						case cmdLog://"log:off" , "log:on" , "log:dump"
						{
							uint8_t lg = logNone;
							if (strstr(uk, "off")) {
								lg = logOff;
							} else if (strstr(uk, "on")) {
								lg = logOn;
							} else if (strstr(uk, "dump")) {
								lg = logDump;
							}
							qcmd.attr = lg;
							cmd_flag = 1;
						}
						break;
					}
					qcmd.cmd = idx;
					if (check) {
						if ((nandAdr + nandLen) >= (total_bytes + devAdr)) {
							nandLen = total_bytes - nandAdr - 1;
						}
						cmd_flag = 1;
					}
					//
					if (cmd_flag) {
						if ((qStat = osMessageQueuePut(myQueHandle, (void *)&qcmd, 5, 0)) != osOK) devError |= devQUE;
					}
					//
				}
			}
			ruk = 0;
			*rxBuf = '\0';
		}

		HAL_UART_Receive_IT(huart, &rxByte, 1);
	}
}
//-------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {// portLOG - log
		uartRdy = true;
	}
}
//-------------------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
		spiRdy = true;
	}
}
//-------------------------------------------------------------------------------------------
/*
void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);
	if (hspi->Instance == SPI1) {//hdma_spi1_tx) {
		spiRdy = 1;
	}
}
*/
//-------------------------------------------------------------------------------------------
void showBuf(uint8_t type, bool rd, uint32_t adr, uint32_t len, const uint8_t *buf)
{
int step = 32;
uint32_t ind = 0;
uint32_t max_ind = len;

	if (type == 2) {
		if (rd) {
			ind = adr & (chipConf.PageSize - 1);// - devAdr;
			max_ind = chipConf.PageSize;
		} else ind = max_ind;
	}
	if (ind < max_ind) {
		bool done = false;
		uint32_t ix = 0, sch = len / step;
		if (len % step) sch++;
		stx[0] = '\0';
		while (!done) {
			sprintf(stx+strlen(stx), "%08X ", (unsigned int)adr);
			for (int i = 0; i < step; i++) {
				sprintf(stx+strlen(stx), " %02X", *(buf + i + ind));
				ix++;
				if (ix == len) {
					done = true;
					break;
				}
			}
			strcat(stx, eol);
			adr += step;
			ind += step;
			sch--;
			if (!sch) done = true;
		}
		if (dbg != logOff) Report(0, "%s", stx);
	} else {
		if (dbg != logOff) Report(0, "\tError: ind=%lu max_ind=%lu readed=%d%s", ind, max_ind, rd, eol);
	}
}
//-------------------------------------------------------------------------------------------


/* USER CODE END 4 */

/* USER CODE BEGIN Header_defThread */
/**
  * @brief  Function implementing the defTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_defThread */
void defThread(void *argument)
{
  /* USER CODE BEGIN 5 */




#ifdef SET_SWV
	char stz[MAX_SCR_BUF];
#endif


	HAL_Delay(500);
	if (dbg != logOff) {
		Report(0, "%s", eol);
		Report(1, "%s Старт '%s' memory:%lu/%lu bytes%s", version, __func__, xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE, eol);
	}

	uint8_t next_block_erase = 0;
	uint32_t iBlk, stik;
	uint8_t nand_show = 0;
	bool readed = false;
	char cid[32];
	uint32_t BlockSizeKB = (chipConf.BlockSize * chipConf.PageSize ) / 1024;
	uint32_t PlaneSizeMB = chipConf.PlaneSize / 1024 / 1024;
	uint8_t *bid = (uint8_t *)&nandID.Maker_Id;
	if (nandState == HAL_NAND_STATE_READY) {
		if (nandID.Device_Id == chipIDcode) strncpy(cid, chipID, sizeof(cid));
		                               else strcpy(cid, "UNKNOWN");
		strcpy(stx, "NAND:");
		for (int8_t i = 0; i < sizeof(NAND_IDsTypeDef); i++) sprintf(stx+strlen(stx), " %02X", *(bid + i));
		sprintf(stx+strlen(stx), "\n\tDevice_Id=%02X '%s'\n", nandID.Device_Id, cid);
		sprintf(stx+strlen(stx), "\tPageSize:%lu\n\tSpareAreaSize:%lu\n\tBlockSize:%lu KB\n\tBlockNbr:%lu\n\tPlaneNbr:%lu\n\tPlaneSize:%lu MB",
					chipConf.PageSize,
					chipConf.SpareAreaSize,
					BlockSizeKB,
					chipConf.BlockNbr,
					chipConf.PlaneNbr,
					PlaneSizeMB);
	} else {
			sprintf(stx, "NAND: Error nandStatus='%s'(%d)",
					     nandAllState[nandState & (MAX_NAND_STATE - 1)], nandState);
	}
	if (dbg != logOff) Report(1, "%s%s", stx, eol);


	char screen[MAX_SCR_BUF];
	uint16_t err_color = BLACK;
	ST7789_Fill(0, 0, ST7789_WIDTH - 1, fntKey->height, YELLOW);
	ST7789_Fill(0, ST7789_WIDTH - fntKey->height, ST7789_WIDTH - 1, ST7789_HEIGHT - 1, WHITE);

	sprintf(screen, "NAND : %s", cid);
	mkLineCenter(screen, ST7789_WIDTH / tFont->width);
	sprintf(screen+strlen(screen),
			"PageSize:%lu\nSpareAreaSize:%lu\nBlockSize:%lu KB\nBlockNbr:%lu\nPlaneNbr:%lu\nPlaneSize:%lu MB",
			chipConf.PageSize,
			chipConf.SpareAreaSize,
			BlockSizeKB,
			chipConf.BlockNbr,
			chipConf.PlaneNbr,
			PlaneSizeMB);
	if (cb_nandCounter) sprintf(screen+strlen(screen), "\nCallBack:%lu", cb_nandCounter);
	ST7789_WriteString(0,
					   tFont->height + (tFont->height * 0.85),
					   screen,
					   *tFont,
					   ~back_color,
					   back_color);
	ipsOn(1);

#ifdef SET_SMALL_FS
	bool mnt = false;

	#ifdef SET_FS_TEST
		fs_err = io_fs_init();
		Report(1, "Mount device '%s' start%s", cid, eol);
		fs_err = io_fs_mount();
		if (fs_err) {
			sprintf(stx, "Mount Error '%s'(%ld)", str_fsErr(fs_err), fs_err);
		} else {
			mnt = true;
			sprintf(stx, "Mount device '%s' OK", cid);
		}
		Report(1, "%s%s", stx, eol);
		if (fs_err) {
			Report(1, "Format...%s", eol);
			fs_err = io_fs_format();
			if (fs_err) {
				Report(1, "Format Error '%s'(%d)%s", str_fsErr(fs_err), fs_err, eol);
			}
		}
		if (!fs_err) {
			Report(1, "Mount device '%s' start%s", cid, eol);
			fs_err = io_fs_mount();
			if (fs_err) {
				printf(stx, "Mount Error '%s'(%d)%s", str_fsErr(fs_err), fs_err);
			} else {
				mnt = true;
				printf(stx, "Mount device '%s' OK%s", cid);
			}
			Report(1, "%s%s", stx, eol);
		}
	#endif
	#ifdef SET_NAND_TEST
		uint32_t page_size = chipConf.PageSize, pg = 0, i;
		Report(0, "----------------------------------------%s", eol);
		bool ok = true;
		io_nand_block_erase(pg);
		io_nand_read(pg, rdBuf, page_size, 0);
		for (i = 0; i < page_size; i++) {
			if (rdBuf[i] != EMPTY) {
				Report(1, "Error erase block #%u at: %d%s", pg, i, eol);
				ok = false;
				break;
			}
		}
		if (ok) Report(1, "Erase block #%u OK%s-------------------%s", pg, eol, eol);
		//
		NAND_AddressTypeDef nan;
		for (pg = 0 * page_size; pg < 5 * page_size; pg += page_size) {
			nan = io_uint32_to_flash_adr(pg);
			Report(1, "Test address: %u (page:%u block:%u plane:%u)%s", pg, nan.Page, nan.Block, nan.Plane, eol);
			for (i = 0; i < page_size; i++) rdBuf[i] = i;

			Report(0, "Write buff...");
			for (i = 0; i < page_size; i++) {
				rdBuf[i] = i;
				//printf("%#x ", buff[i]);
				//if (i % 16 == 0 && i != 0) printf("\r\n");
			}
			io_nand_write(pg, rdBuf, page_size, 0);
			Report(0, " OK%s", eol);

			memset(rdBuf, 0, page_size);
			io_nand_read(pg, rdBuf, page_size, 0);

			ok = true;
			Report(0, "Read buffer and check...");
			for (i = 0; i < page_size; i++) {
				if (rdBuf[i] != i % 256) {
					Report(0, " Error read at: %d%s", i, eol);
					ok = false;
					break;
				}
				//printf("%#x ", buff[i]);
				//if (i % 16 == 0 && i != 0) printf("\r\n");
			}
			if (ok) Report(0, " OK...%s--------------------%s", eol);
		}
	#endif
#endif

	bool loop = true;
	bool led = false;
	uint32_t tmr = get_tmr(1);

	s_qcmd qcmd = {0};
	uint8_t prio = 0;
	osStatus_t qs = osOK;

  /* Infinite loop */

	while (loop) {

		if (check_tmr(tmr)) {
			tmr = get_tmr(1);
			//
			sec2str(screen);
#ifdef SET_SWV
			strcpy(stz, screen);
#endif
			ST7789_WriteString(8, 0, mkLineCenter(screen, ST7789_WIDTH / fntKey->width), *fntKey, BLUE, YELLOW);

			sprintf(screen, "Error: 0x%04X", devError);
			if (devError) err_color = RED; else err_color = BLACK;
			ST7789_WriteString(0, ST7789_WIDTH - fntKey->height, mkLineCenter(screen, ST7789_WIDTH / fntKey->width), *fntKey, err_color, WHITE);
			//
#ifdef SET_SWV
			//puts("Second...");
			printf("[%s] %s%s", __func__, stz, eol);
#endif
			//

			if (qStat != 0) {
				if (qs != qStat) {
					if (dbg != logOff) Report(1, "OS: %s%s", get_qStat(qStat), eol);
					qs = qStat;
				}
			}

			if (devError) led = true; else led = false;
			errLedOn(led);

		}

		if ((qStat = osMessageQueueGet(myQueHandle, (void *)&qcmd, &prio, 5)) != osOK) {
			if (qs != qStat) qs = qStat;
			if (qStat != osErrorTimeout) {
				devError |= devQUE;
				if (dbg != logOff) Report(1, "OS: %s%s", get_qStat(qStat), eol);
			}
		} else {
			sprintf(screen, "Cmd: %s", str_cmds[qcmd.cmd]);
			ST7789_WriteString(0, ST7789_WIDTH - (fntKey->height << 1),
							   mkLineCenter(screen, ST7789_WIDTH / fntKey->width),
							   *fntKey,
							   CYAN,
							   BLACK);
			//
			if (dbg > logOn)
				Report(1, "Command(%u.%u): '%s'%s", qcmd.cmd, qcmd.attr, str_cmds[qcmd.cmd], eol);
			//
			nand_show = 0;
			switch (qcmd.cmd) {
				case cmdHelp:
					sprintf(stx, "Support next commands:%s", eol);
					for (uint8_t i = 0; i < MAX_CMDS; i++) sprintf(stx+strlen(stx), "\t'%s'%s", s_cmds[i], eol);
					Report(1, "%s", stx);
				break;
				case cmdRestart:
					loop = false;
		  		break;
				case cmdInfo:
					if (dbg != logOff) {
						strcpy(stx, "NAND:");
						//uint8_t *bid = (uint8_t *)&nandID.Maker_Id;
						for (int8_t i = 0; i < sizeof(NAND_IDsTypeDef); i++) sprintf(stx+strlen(stx), " %02X", *(bid + i));
						sprintf(stx+strlen(stx), "\n\tDevice_Id=%02X '%s'\n", nandID.Device_Id, cid);
						sprintf(stx+strlen(stx), "\tPageSize:%lu\n\tSpareAreaSize:%lu\n\tBlockSize:%lu KB\n\tBlockNbr:%lu\n\tPlaneNbr:%lu\n\tPlaneSize:%lu MB",
											chipConf.PageSize,
											chipConf.SpareAreaSize,
											BlockSizeKB,
											chipConf.BlockNbr,
											chipConf.PlaneNbr,
											PlaneSizeMB);
						Report(1, "%s%s", stx, eol);
					}
				break;
				case cmdEpoch:
					if (!qcmd.attr) {//set date&time
						set_Date(epoch);
					} else {//show date&time
						sec2str(stx);
						if (dbg != logOff) Report(0, "%s <- Current date&time%s", stx, eol);
					}
				break;
				case cmdLog:
					if (qcmd.attr < logNone) {
						Report(1, "Set log level to '%s'(%u)%s", get_logName(qcmd.attr), qcmd.attr, eol);
						dbg = qcmd.attr;
					} else {
						Report(1, "Current log level is '%s'(%u)%s", get_logName(dbg), dbg, eol);
					}
				break;
				case cmdRead:
					if (rdBuf) {
						io_nand_read(nandAdr - devAdr, rdBuf, nandLen, 0);
						if (!(devError & devNAND)) {
							nand_show = 1;
							readed = true;
						}
					}
				break;
				case cmdNext:
					if (dbg != logOff) Report(1, "Read next nand adr:0x%X len:%lu%s", nandAdr, nandLen, eol);
					if (rdBuf) nand_show = 2;
				break;
				case cmdErase:
					if (!qcmd.attr) {
						uint32_t bk = nandBlk * chipConf.PageSize;
						io_nand_block_erase(bk);
						if (dbg != logOff) {
							if (devError & devNAND) {
								Report(1, "Erase nand block:%lu addr:%u Error !%s", nandBlk, bk, eol);
							} else {
								Report(1, "Erase nand block:%lu addr:%u Ok !%s", nandBlk, bk, eol);
							}
						}
					} else {
						iBlk = 0;
						next_block_erase = 1;
						if (dbg != logOff) Report(1, "Erase chip ");
						stik = HAL_GetTick();
					}
				break;
				case cmdCheck:
				{
					uint32_t adr = nandPage * chipConf.PageSize;//nand_PageToBlock(nandPage);
					if (!pageIsEmpty(adr)) {
						if (dbg != logOff) Report(1, "Page:%lu in addr:%lu Not empty%s", nandPage, adr, eol);
					} else {
						if (dbg != logOff) Report(1, "Page:%lu in addr:%lu is Empty%s", nandPage, adr, eol);
					}
				}
				break;
				case cmdWrite:
					if (wrBuf) {
						uint32_t wadr = nandAdr - devAdr;
						if (!pageIsEmpty(wadr)) {
							io_nand_block_erase(wadr);
							sprintf(stx, "Erase nand addr:%lu done", wadr);
						} else {
							sprintf(stx, "Addr:%lu is Empty", wadr);
						}
						if (dbg != logOff) Report(1, "%s%s", stx, eol);
						memset(wrBuf, EMPTY, chipConf.PageSize);
						uint32_t ofs = 0;//(nandAdr - devAdr) % chipConf.PageSize;
						memset(wrBuf /* + ofs*/, nandByte, nandLen);
						////showBuf(1, false, devAdr, 512,/*nandAdr, nandLen,*/ wrBuf);
						//if (NAND_Write_Page_8b(nandPort, &addr, wrBuf, nandLen, ofs) != HAL_OK) devError |= devNAND;
						//if (dbg != logOff) Report(1, "Write nand adr:0x%X byte:0x%02X len:%lu ofs:%lu (page:%lu blk:%lu)%s",
						//	      	  	  	  	  	  nandAdr, nandByte, nandLen, ofs, addr.Page, addr.Block, eol);
						io_nand_write(wadr, wrBuf, nandLen, ofs);
						if (dbg != logOff) Report(1, "Write nand adr:0x%X byte:0x%02X len:%lu ofs:%lu%s",
							      	  	  	  	  	  nandAdr, nandByte, nandLen, ofs, eol);
					}
				break;
			}
			if (nand_show) {
				showBuf(nand_show, readed, nandAdr, nandLen, rdBuf);
			}
		}

		// Erase all pages
		if (next_block_erase) {
			io_nand_block_erase(iBlk);
			if (devError & devNAND) {
				next_block_erase = 0;
			} else {
				if (!(iBlk % (chipConf.PageSize * 1024))) Report(0, ".");
				if (iBlk >= (chipConf.PlaneSize * chipConf.PlaneNbr)) next_block_erase = 0;
			}
			iBlk += chipConf.PageSize;
			if (!next_block_erase) {
				if (dbg != logOff) Report(0, " %lu blocks (%lu sec)%s", iBlk, (HAL_GetTick() - stik) / 1000, eol);
			}
		}
		//
		osDelay(5);
	}

	if (wrBuf) free(wrBuf);
	if (rdBuf) free(rdBuf);

#ifdef SET_SMALL_FS
	if (mnt) fs_err = io_fs_unmount();
#endif

	if (dbg != logOff) Report(1, "%s Стоп '%s' memory:%lu/%lu bytes ...%s", version, __func__, xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE, eol);
	osDelay(250);

	NVIC_SystemReset();

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM2) {
	  if ((get_msCounter() & 3) == 3) {//seconda
		  inc_secCounter();
		  HAL_GPIO_TogglePin(LED_TIK_GPIO_Port, LED_TIK_Pin);
	  }
	  inc_msCounter();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

	devError |= devSYS;
	errLedOn(true);

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
