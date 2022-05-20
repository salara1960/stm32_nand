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
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
const char *version = "ver.0.8 (20.05.2022)";



const char *eol = "\r\n";
const char *s_restart = "restart";
const char *s_epoch   = "epoch:";
const char *s_cmds[MAX_CMDS] = {"read:", "next", "write:", "erase:"};
s_flags flags = {0};
uint32_t devError;

volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t msCounter = 0;//period 250ms
static char txBuf[MAX_UART_BUF] = {0};
static char rxBuf[MAX_UART_BUF] = {0};
uint8_t rxByte = 0;
uint16_t ruk = 0;
bool uartRdy = true;
bool spiRdy = true;
bool setDate = false;
//1652998677;//1652445122;//1652361110;//1652296740;//1652042430;//1652037111;
uint32_t epoch = 1653082240;//1653055492;
uint8_t tZone = 0;//2;

SPI_HandleTypeDef *ipsPort = &hspi1;
TIM_HandleTypeDef *timePort = &htim2;
UART_HandleTypeDef *logPort = &huart3;
NAND_HandleTypeDef *nandPort = &hnand1;

const FontDef *fntKey = &Font_16x26;
const FontDef *tFont = &Font_11x18;
uint16_t back_color = BLACK;


uint32_t devAdr = 0;
uint32_t nandAdr = 0;
uint16_t nandLen = 0;
uint32_t nandBlk = 0;
uint8_t nandByte = 0xff;
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
uint8_t *rdBuf = NULL;
uint8_t *wrBuf = NULL;

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
HAL_StatusTypeDef HAL_NAND_Read_IDs(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID);
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void errLedOn(bool on);
void set_Date(uint32_t usec);
uint32_t getSecRTC(RTC_HandleTypeDef *hrtc);
int sec2str(char *st);
uint8_t Report(uint8_t addTime, const char *fmt, ...);
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
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LED_TIK_GPIO_Port, LED_TIK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LED_TIK_GPIO_Port, LED_TIK_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
  }

  // start timer2 + interrupt
  HAL_TIM_Base_Start_IT(timePort);

  HAL_UART_Receive_IT(logPort, &rxByte, 1);

  set_Date(epoch);

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
/*
  char st[128] = {0};
  uint32_t tmsec = get_tmr(2);
  while (1) {
	  if (check_tmr(tmsec)) {
		  tmsec = get_tmr(2);
		  sec2str(st);
		  printf("%s%s", st, eol);
	  }
	  HAL_Delay(100);
  }
*/
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
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
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
  huart3.Init.BaudRate = 115200;
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
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
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
  hnand1.Config.BlockSize = 131072;
  hnand1.Config.BlockNbr = 1024;
  hnand1.Config.PlaneNbr = 1;
  hnand1.Config.PlaneSize = 134217728;
  hnand1.Config.ExtraCommandEnable = ENABLE;
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 252;
  ComSpaceTiming.WaitSetupTime = 252;
  ComSpaceTiming.HoldSetupTime = 252;
  ComSpaceTiming.HiZSetupTime = 252;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 252;
  AttSpaceTiming.WaitSetupTime = 252;
  AttSpaceTiming.HoldSetupTime = 252;
  AttSpaceTiming.HiZSetupTime = 252;

  if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  if (HAL_NAND_Read_IDs(&hnand1, &nandID) == HAL_OK) {

	  nandState = HAL_NAND_GetState(&hnand1);

	  memcpy((uint8_t *)&chipConf, (uint8_t *)&hnand1.Config, sizeof(s_chipConf));

	  if ((chipConf.PageSize > 0) && (chipConf.PageSize <= MAX_NAND_BUF)) {
		  rdBuf = (uint8_t *)calloc(1, chipConf.PageSize);
		  if (!rdBuf) devError |= devMEM;
	  } else {
		  devError |= devNAND;
	  }

  }


  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef HAL_NAND_Read_IDs(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID)
{
__IO uint32_t data = 0U;
__IO uint32_t data1 = 0U;
uint32_t deviceaddress = 0U;

	__HAL_LOCK(hnand);

	if(hnand->State == HAL_NAND_STATE_BUSY) return HAL_BUSY;

	if(hnand->Init.NandBank == FMC_NAND_BANK2) {
		deviceaddress = NAND_DEVICE1;
	} else {
		deviceaddress = NAND_DEVICE2;
	}
	devAdr = deviceaddress;

	hnand->State = HAL_NAND_STATE_BUSY;

	/* Send Read ID command sequence */
	*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA))  = NAND_CMD_READID;
	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;

	/* Read the electronic signature from NAND flash */
	if (hnand->Init.MemoryDataWidth == FSMC_NAND_PCC_MEM_BUS_WIDTH_8) {
		data = *(__IO uint32_t *)deviceaddress;
		data1 = *((__IO uint32_t *)deviceaddress + 4U);

		/* Return the data read */
		pNAND_ID->Maker_Id  = ADDR_1ST_CYCLE(data);
		pNAND_ID->Device_Id = ADDR_2ND_CYCLE(data);
		pNAND_ID->Third_Id  = ADDR_3RD_CYCLE(data);
		pNAND_ID->Fourth_Id = ADDR_4TH_CYCLE(data);
		pNAND_ID->Plane_Id  = ADDR_1ST_CYCLE(data1);

		hnand->State = HAL_NAND_STATE_READY;
	}

	__HAL_UNLOCK(hnand);

	return HAL_OK;
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
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	struct tm ts;
	time_t ep = (time_t)usec;

	gmtime_r(&ep, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
	else {
		sTime.Hours   = ts.tm_hour + tZone;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
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

		ret = sprintf(st, "%lu.%02lu:%02lu:%02lu ", day, hour, min, sec);
	} else {
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) devError |= devRTC;
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) devError |= devRTC;
			else {
				ret = sprintf(st, "%02u.%02u %02u:%02u:%02u ",
								sDate.Date, sDate.Month,
								sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}

	return ret;
}
//-------------------------------------------------------------------------------------------
uint8_t Report(uint8_t addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;
char *buf = &txBuf[0];

	*buf = '\0';
	if (addTime) {
		dl = sec2str(buf);
		strcat(buf, "| ");
		dl += 2;
	}

	va_start(args, fmt);
	vsnprintf(buf + dl, len - dl, fmt, args);

	uartRdy = false;
	if (HAL_UART_Transmit_DMA(logPort, (uint8_t *)buf, strlen(buf)) != HAL_OK) devError |= devUART;
	while (!uartRdy) HAL_Delay(1);

	va_end(args);

	return 0;
}
//------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {// logPort - log
		rxBuf[ruk++] = (char)rxByte;
		if (rxByte == 0x0a) {//end of line
			char *uk = NULL;
			bool check = false;
			if (strstr(rxBuf, s_restart)) {
				flags.restart = 1;
			} else if ((uk = strstr(rxBuf, s_epoch))) {
				uk += strlen(s_epoch);
				if (*uk != '?') {
					if (strlen(uk) < 10) setDate = false;
					else {
						uint32_t ep = (uint32_t)atol(uk);
						if (ep > epoch) {
							epoch = ep;
							flags.time_set = 1;
						}
					}
				} else {
					setDate = true;
					flags.time_show = 1;
				}
			} else {
				int8_t idx = -1;
				for (int8_t i = 0; i < MAX_CMDS; i++) {
					if ((uk = strstr(rxBuf, s_cmds[i]))) {//const char *s_cmds = "read:0x4549ABBB:256";
						                                  //"next";
														  //"write:0x0:256:0xf0"
														  //"erase:"
						idx = i;
						break;
					}
				}
				if (uk == rxBuf) {
					nandLen = 256;
					uk += strlen(s_cmds[idx]);
					char *uki = NULL;
					switch (idx) {
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
							flags.cmd = cmdRead;
						break;
						case cmdNext://"next";
							if (nandAdr < devAdr) nandAdr = devAdr;
							nandAdr += nandLen;
							check = true;
							flags.cmd = cmdNext;
						break;
						case cmdWrite://"write:0x0:256:0xf0"
						break;
						case cmdErase://"erase:0" //erase:block_number from 0..1023
						{
							uint32_t blk = atol(uk);
							if (blk < chipConf.BlockNbr) {
								nandBlk = blk;
								flags.cmd = cmdErase;
								flags.cmd_flag = 1;
							}
						}
						break;
					}
					if (check) {
						flags.cmd_flag = 1;
						if ((nandAdr + nandLen) >= (chipConf.PlaneSize + devAdr)) {
							nandLen = chipConf.PlaneSize - nandAdr - 1;
						}
					}
				}
			}
			ruk = 0;
			memset(rxBuf, 0, MAX_UART_BUF);
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
void HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand)
{
	if (hnand->Instance == FSMC_NAND_DEVICE) {
		cb_nandCounter++;
	}
}
//-------------------------------------------------------------------------------------------
uint32_t nand_PageToBlock(uint32_t page)
{
    return (page * chipConf.PageSize) / chipConf.BlockSize;
}
uint32_t nand_BlockToPage(uint32_t blk)
{
	return (blk * chipConf.BlockSize) / chipConf.PageSize;
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

	*(uint8_t *)&flags = 0;

	char *stx = (char *)calloc(1, MAX_UART_BUF);
#ifdef SET_SWV
	char stz[MAX_TMP_SIZE];
#endif


	HAL_Delay(1000);
	Report(0, "%s", eol);
	Report(1, "%s Старт '%s' memory:%lu/%lu bytes%s", version, __func__, xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE, eol);


	uint8_t nand_show = 0;
	bool readed = false;
	char cid[32];
	if (nandState == HAL_NAND_STATE_READY) {
		if (nandID.Device_Id == chipIDcode) strncpy(cid, chipID, sizeof(cid));
		                               else strcpy(cid, "UNKNOWN");
		strcpy(stx, "NAND:");
		uint8_t *bid = (uint8_t *)&nandID.Maker_Id;
		for (int8_t i = 0; i < sizeof(NAND_IDsTypeDef); i++) sprintf(stx+strlen(stx), " %02X", *(bid + i));
		sprintf(stx+strlen(stx),
				"\n\tMakerID=%02X\n\tDevice_Id=%02X '%s'\n\tThird_Id=%02X\n\tFourth_Id=%02X\n\tPlane_Id=%02X",
			    nandID.Maker_Id, nandID.Device_Id, cid, nandID.Third_Id, nandID.Fourth_Id, nandID.Plane_Id);
	} else {
			sprintf(stx,
					"NAND: Error nandStatus='%s'(%d)",
					nandAllState[nandState & (MAX_NAND_STATE - 1)], nandState);
	}
	Report(1, "%s%s", stx, eol);


	//fntKey = &Font_16x26;
	//tFont = &Font_11x18;
	//ipsOn(0);
	uint16_t err_color = BLACK;
	ST7789_Fill(0, 0, ST7789_WIDTH - 1, fntKey->height, YELLOW);
	ST7789_Fill(0, ST7789_WIDTH - fntKey->height, ST7789_WIDTH - 1, ST7789_HEIGHT - 1, WHITE);

	sprintf(stx, "NAND : %s", cid);
	mkLineCenter(stx, ST7789_WIDTH / tFont->width);
	//sprintf(stx+strlen(stx), "Maker:0x%02X\nchipID:0x%02X\n3-rd:0x%02X\n4-th:0x%02X\n5-th:0x%02X",
	//		                 nandID.Maker_Id, nandID.Device_Id, nandID.Third_Id, nandID.Fourth_Id, nandID.Plane_Id);
	sprintf(stx+strlen(stx),
			"PageSize:%lu\nSpareAreaSize:%lu\nBlockSize:%lu KB\nBlockNbr:%lu\nPlaneNbr:%lu\nPlaneSize:%lu MB",
			chipConf.PageSize,
			chipConf.SpareAreaSize,
			chipConf.BlockSize / 1024,
			chipConf.BlockNbr,
			chipConf.PlaneNbr,
			chipConf.PlaneSize / 1024 / 1024);
	if (cb_nandCounter) sprintf(stx+strlen(stx), "\nCallBack:%lu", cb_nandCounter);
	ST7789_WriteString(0,
					   tFont->height + (tFont->height * 0.85),
					   stx,
					   *tFont,
					   ~back_color,
					   back_color);
	ipsOn(1);

	bool led = false;
	uint32_t tmr = get_tmr(1);

  /* Infinite loop */

  while (1) {

	  if (check_tmr(tmr)) {
		  tmr = get_tmr(1);
		  //
		  sec2str(stx);
#ifdef SET_SWV
		  strcpy(stz, stx);
#endif
		  ST7789_WriteString(8, 0, mkLineCenter(stx, ST7789_WIDTH / fntKey->width), *fntKey, BLUE, YELLOW);

		  sprintf(stx, "Error: 0x%04X", (unsigned int)devError);
		  if (devError) err_color = RED; else err_color = BLACK;
		  ST7789_WriteString(0, ST7789_WIDTH - fntKey->height, mkLineCenter(stx, ST7789_WIDTH / fntKey->width), *fntKey, err_color, WHITE);
		  //
#ifdef SET_SWV
		  //puts("Second...");
		  printf("[%s] %s%s", __func__, stz, eol);
#endif
	  }

	  if (devError) led = true; else led = false;
	  errLedOn(led);

	  nand_show = 0;

	  if (flags.restart) {
		  flags.restart = 0;
		  break;
	  } else if (flags.time_set) {
		  flags.time_set = 0;
		  set_Date(epoch);
	  } else if (flags.time_show) {
		  flags.time_show = 0;
		  sec2str(stx);
		  Report(0, "Current date&time -> %s%s", stx, eol);
	  } else if (flags.cmd_flag) {
		  flags.cmd_flag = 0;
		  switch (flags.cmd) {
		  	  case cmdRead:
		  	  {
		  		  uint32_t p = (nandAdr - devAdr) / chipConf.PageSize;
		  		  NAND_AddressTypeDef addr = {
		  		      .Page = p,
					  .Plane = 1,
					  .Block = nand_PageToBlock(p)
		  		  };
		  		  Report(1, "Read nand adr:0x%X len:%lu (page:%lu blk:%lu)%s",
		  				    nandAdr, nandLen, addr.Page, addr.Block, eol);
		  		  if (rdBuf) {
		  			  if (HAL_NAND_Read_Page_8b(nandPort, &addr, rdBuf, 1) == HAL_OK) {
		  				  nand_show = 1;
		  				  readed = true;
		  			  } else devError |= devNAND;
		  		  }
		  	  }
		  	  break;
		  	  case cmdNext:
		  		  Report(1, "Read next nand adr:0x%X len:%lu%s",
		  				    nandAdr, nandLen, eol);
		  		  if (rdBuf) nand_show = 2;
		  	  break;
		  	  case cmdErase:
		  	  {
		  		  NAND_AddressTypeDef addr = {
		  			  .Page = nand_BlockToPage(nandBlk),
					  .Plane = 1,
				      .Block = nandBlk
		  		  };
		  		  Report(1, "Erase nand page:%lu blk:%lu...",
		  				    addr.Page, addr.Block);
		  		  if (rdBuf) {
		  			  if (HAL_NAND_Erase_Block(nandPort, &addr) != HAL_OK) devError |= devNAND;
		  		  }
		  		  Report(0, "%s", eol);
		  	  }
		  	  break;
		  }
		  if (nand_show) {
			  uint32_t adr = nandAdr;
			  int step = 32;
			  uint32_t ind = 0;
			  uint32_t max_ind = nandLen;
			  if (nand_show == 2) {
				  if (readed) {
					  ind = adr & (chipConf.PageSize - 1);// - devAdr;
					  max_ind = chipConf.PageSize;
				  } else ind = max_ind;
			  }
			  if (ind < max_ind) {
				  bool done = false;
				  uint32_t sch = nandLen / step;
				  stx[0] = '\0';
				  while (!done) {
					  sprintf(stx+strlen(stx), "%08X ", (unsigned int)adr);
					  for (int i = 0; i < step; i++) sprintf(stx+strlen(stx), " %02X", rdBuf[i + ind]);
					  strcat(stx, eol);
					  adr += step;
					  ind += step;
					  sch--;
					  if (!sch) done = true;
				  }
				  Report(0, "%s", stx);
			  } else {
				  Report(0, "\tError: ind=%lu max_ind=%lu readed=%d%s", ind, max_ind, readed, eol);
			  }
		  }
	  }
	  osDelay(250);
  }

  if (rdBuf) free(rdBuf);
  if (stx) free(stx);


  Report(1, "%s Стоп '%s' memory:%lu/%lu bytes ...%s", version, __func__, xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE, eol);
  osDelay(1000);

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

