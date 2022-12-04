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
// #include "usb_host.h"// 2022.11.13 Uncommenting this out b/c I think I need to call the stm32f7xx.h chain of calls somehow and it was happening in a HAL, but it could happen here too!
// #include "main.h"
#include "stdint.h"
#include "cmsis_gcc.h"
#include "system_stm32f7xx.h"
#include <stdio.h> // Needed to use switch-case I think

void JHM_Delay(uint32_t Delay_copy);
void JHM_TxPinWrite(uint32_t* GPIO_Port_OI_Addy, uint32_t PinWithinPort, uint8_t bit2write);

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
//ADC_HandleTypeDef hadc3;
//
//CRC_HandleTypeDef hcrc;
//
//DCMI_HandleTypeDef hdcmi;
//
//DMA2D_HandleTypeDef hdma2d; // JHM needed to comment out some interrupt HAL calls to no longer need this
//
//ETH_HandleTypeDef heth;
//
//I2C_HandleTypeDef hi2c1;
//I2C_HandleTypeDef hi2c3;
//
//LTDC_HandleTypeDef hltdc; // JHM needed to comment out some interrupt HAL calls to no longer need this
//
//QSPI_HandleTypeDef hqspi;
//
//RTC_HandleTypeDef hrtc;
//
//SAI_HandleTypeDef hsai_BlockA2;
//SAI_HandleTypeDef hsai_BlockB2;
//
//SD_HandleTypeDef hsd1;
//
//SPDIFRX_HandleTypeDef hspdif;
//
//SPI_HandleTypeDef hspi2;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim5;
//TIM_HandleTypeDef htim8;
//TIM_HandleTypeDef htim12;
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart6;
//
//SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
volatile uint32_t uwTick_copy;
uint32_t uwTickFreq_copy;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void); // 2022.11.13 Oh.. I thought I needed this, so it was in the "No HAL" commits today, but evidently not since it's commented out
//static void MX_ADC3_Init(void);
//static void MX_CRC_Init(void);
//static void MX_DCMI_Init(void);
//static void MX_DMA2D_Init(void);
//static void MX_ETH_Init(void);
//static void MX_FMC_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_I2C3_Init(void);
//static void MX_LTDC_Init(void);
//static void MX_QUADSPI_Init(void);
//static void MX_RTC_Init(void);
//static void MX_SAI2_Init(void);
// static void MX_SDMMC1_SD_Init(void);
//static void MX_SPDIFRX_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_TIM5_Init(void);
//static void MX_TIM8_Init(void);
//static void MX_TIM12_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_USART6_UART_Init(void);
//void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  __disable_irq();

  // HAL_Init();

//  //BLOCK0 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
//  /* Set Interrupt Group Priority */
//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

// JHM CODE (Working as of 2022.11.12 !)
  // SCB -> AIRCR Setting
  uint32_t* SCB_AIRCR_OI = (uint32_t *)0XE000ED0CUL; // Was 0XE000E*0*0CUL, empirically is 0XE000ED0CUL
  uint32_t SCB_AIRCR_tmp = *SCB_AIRCR_OI;
  SCB_AIRCR_tmp &= ((uint32_t)(0xFFFFUL << 16 | 7UL << 8U)); // Might need to check types are good
  SCB_AIRCR_tmp = (SCB_AIRCR_tmp | ((uint32_t)0x5FAUL << 16U) | (3U << 8U));
  //  *SCB_AIRCR_OI = SCB_AIRCR_tmp; // Empirically expected to be 0x5FA0300 (same?? No... it's 0xFFFF0300)
  *SCB_AIRCR_OI = 0x5FA0300UL;

//  //BLOCK0 HAL_InitTick(TICK_INT_PRIORITY);
//  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
//  HAL_InitTick(TICK_INT_PRIORITY);

//	#include "stm32f7xx_hal.h"
//	#include "stm32f7xx_hal_tim.h"
//  	TIM_HandleTypeDef        htim6_OI; // JHM added 2022.11.12 in debug
//  	// htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 1, AutoReloadPreload = 48}, Channel = (HAL_TIM_ACTIVE_CHANNEL_3 | HAL_TIM_ACTIVE_CHANNEL_6), hdma = {0x0, 0x8000359 <main+336>, 0x80003ba <main+434>, 0x21000000, 0xf0, 0xfa, 0x1}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_BUSY, ChannelState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, ChannelNState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, DMABurstState = HAL_DMA_BURST_STATE_READY}
//  	htim6_OI.Init.RepetitionCounter = 0;
//  	htim6_OI.Init.AutoReloadPreload = 0;
//  	htim6_OI.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
//  	htim6_OI.hdma[0] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[1] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[2] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[3] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[4] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[5] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.hdma[6] = (DMA_HandleTypeDef *) 0x0;
//  	htim6_OI.State = HAL_TIM_STATE_RESET;
//  	htim6_OI.ChannelState[0] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelState[1] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelState[2] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelState[3] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelState[4] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelState[5] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelNState[0] = HAL_TIM_CHANNEL_STATE_RESET;
//  	htim6_OI.ChannelNState[1] = HAL_TIM_CHANNEL_STATE_RESET;
//    htim6_OI.ChannelNState[2] = HAL_TIM_CHANNEL_STATE_RESET;
//    htim6_OI.ChannelNState[3] = HAL_TIM_CHANNEL_STATE_RESET;
//    htim6_OI.DMABurstState = HAL_DMA_BURST_STATE_RESET;


//    RCC_ClkInitTypeDef    clkconfig;
	uint32_t              uwTimclock = 0;
//	uint32_t              uwPrescalerValue = 0;
//	uint32_t              pFLatency;
	/*Configure the TIM6 IRQ priority */
//	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TICK_INT_PRIORITY ,0); // 2022.11.13, weird I thought I'd need this one, but priority must be "good enough"

//	/* Enable the TIM6 global Interrupt */
//	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	//BLOCK1 HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	//NIVC -> ISER Setting
	uint32_t* NVIC_ISER_OI = (uint32_t*)0xE000E104UL; // Empirically 0xe000e104 (same)
	*NVIC_ISER_OI = (uint32_t)(1U << 22U); // Empirically 0x400000 (same)

//	/* Enable TIM6 clock */
//	__HAL_RCC_TIM6_CLK_ENABLE();

	//BLOCK1 __HAL_RCC_TIM6_CLK_ENABLE();
	//RCC -> APB1ENR for TIM6EN
	uint32_t* RCC_APB1ENR_OI = (uint32_t *)0x40023840UL; // Empirically 0x40023840 (same)
	*RCC_APB1ENR_OI |= (uint32_t)(1U << 4U); // Empirically 16 (same)

	/* Get clock configuration */
//	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency); // 2022.11.13, weird I thought I'd need this one.

//	HAL_InitTick(TICK_INT_PRIORITY);

	/* Compute TIM6 clock */
//	uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

//	uwTimclock = 2*(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
	uwTimclock = 2*(SystemCoreClock); // JHM Looks like we just need to call this "SystemCoreClock" somewhere. It's not HAL - defined in system_stm32f7xx.c
	uwTimclock = 16000000;
	/* Compute the prescaler value to have TIM6 counter clock equal to 1MHz */
//	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);
//
//	/* Initialize TIM6 */
//	htim6_OI.Instance = TIM6;
//
//	/* Initialize TIMx peripheral as follow:
//	+ Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
//	+ Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
//	+ ClockDivision = 0
//	+ Counter direction = Up
//	*/
//	htim6_OI.Init.Period = (1000000U / 1000U) - 1U;
//	htim6_OI.Init.Prescaler = uwPrescalerValue;
//	htim6_OI.Init.ClockDivision = 0;
//	htim6_OI.Init.CounterMode = TIM_COUNTERMODE_UP;
    // JHM htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 0, AutoReloadPreload = 0}, Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED, hdma = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_RESET, ChannelState = {HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET}, ChannelNState = {HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET}, DMABurstState = HAL_DMA_BURST_STATE_RESET}
	// HAL htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 0, AutoReloadPreload = 0}, Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED, hdma = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_RESET, ChannelState = {HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET}, ChannelNState = {HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_RESET}, DMABurstState = HAL_DMA_BURST_STATE_RESET}
    // HAL htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 0, AutoReloadPreload = 0}, Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED, hdma = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_READY, ChannelState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, ChannelNState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, DMABurstState = HAL_DMA_BURST_STATE_READY} // Changes to the last 4 variables (RESET -> READY)
	// JHM htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 1, AutoReloadPreload = 48}, Channel = (HAL_TIM_ACTIVE_CHANNEL_3 | HAL_TIM_ACTIVE_CHANNEL_6), hdma = {0x0, 0x8000359 <main+336>, 0x80003ba <main+434>, 0x21000000, 0xf0, 0xfa, 0x1}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_BUSY, ChannelState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, ChannelNState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, DMABurstState = HAL_DMA_BURST_STATE_READY}
	// JHM htim6 = {Instance = 0x40001000, Init = {Prescaler = 31, CounterMode = 0, Period = 999, ClockDivision = 0, RepetitionCounter = 1, AutoReloadPreload = 48}, Channel = (HAL_TIM_ACTIVE_CHANNEL_3 | HAL_TIM_ACTIVE_CHANNEL_6), hdma = {0x0, 0x8000359 <main+336>, 0x80003ba <main+434>, 0x21000000, 0xf0, 0xfa, 0x1}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_READY, ChannelState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, ChannelNState = {HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_READY}, DMABurstState = HAL_DMA_BURST_STATE_READY}

//	HAL_TIM_Base_Init(&htim6_OI);
//	HAL_TIM_Base_Start_IT(&htim6_OI);

  // htim CR1
  uint32_t* htim6_CR1_OI = (uint32_t*)0x40001000UL; // Empirically 0x40001000 (same)
  *htim6_CR1_OI = (uint32_t)1UL; // Empirically 1 (same)

  // htim ARR
  uint32_t* htim6_ARR_OI = (uint32_t*)0x4000102CUL; // Empirically 0x4000102c (same)
  *htim6_ARR_OI = (uint32_t)999UL; // Empirically 999 (same)

  // htim PSC
  uint32_t *htim6_PSC_OI = (uint32_t*)0x40001028UL; // Empirically 0x40001028 (same)
  *htim6_PSC_OI = (uint32_t)31UL; // Empirically 31 (same)

  // htim EGR
  uint32_t *htim6_EGR_OI = (uint32_t*)0x40001014UL; // Empirically 0x40001014 (same)
  *htim6_EGR_OI = (uint32_t)1UL; // Check if this set the following addy to 1 and the above addy stays at 0: 0x40001010UL // Empirically 1 (same) // Looks like this is turned to "1" momentarily to update something, then it's 0, so this implementation seems right

  //htim6.Instance = TIM6 within HAL_InitTick() or HAL_InitTick_JHM()
  uint32_t* htim_Instance_OI = (uint32_t*)0x20000028UL; // Empirically 0x20000028 (same) // 2022.11.13, weird was my issue in replacing HAL_InitTick() that is was before the prior htim6 lines and it needed to be after??
  *htim_Instance_OI = 0x40001000UL; // Empirically 0x40001000 (same)

//	HAL_InitTick_JHM(TICK_INT_PRIORITY);

	// Before TIM_Base_SetConfig(): print(*(htim->Instance)) = {CR1 = 1, CR2 = 0, SMCR = 0, DIER = 1, SR = 1, EGR = 0, CCMR1 = 0, CCMR2 = 0, CCER = 0, CNT = 970, PSC = 31, ARR = 999, RCR = 0, CCR1 = 0, CCR2 = 0, CCR3 = 0, CCR4 = 0, BDTR = 0, DCR = 0, DMAR = 0, OR = 0, CCMR3 = 0, CCR5 = 0, CCR6 = 0}
	// Also, print(htim->Instance) = (TIM_TypeDef *) 0x40001000


//	if(HAL_TIM_Base_Init(&htim6) == HAL_OK)
//	{
//	  /* Start the TIM time Base generation in interrupt mode */
//	  HAL_TIM_Base_Start_IT(&htim6); // JM removed the return
//	}
//	// deleted the return here too

//	HAL_InitTick(TICK_INT_PRIORITY);

//  //BLOCK1 HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority ,0);
//  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TICK_INT_PRIORITY, 0); // Changed TickPriority to TICK_INT_PRIORITY

//  // NVIC -> IP Setting
//  uint32_t* NVIC_IP_OI = (uint32_t*)(0xE000E400UL + 54UL); // Might need to check types are good. Empirically 0xE000E436UL (same)
//  *NVIC_IP_OI = (uint8_t)0UL; // Empirically 0UL (same)

//  //BLOCK1 HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

//  //NIVC -> ISER Setting
//  uint32_t* NVIC_ISER_OI = (uint32_t*)0xE000E104UL; // Empirically 0xe000e104 (same)
//  *NVIC_ISER_OI = (uint32_t)(1U << 22U); // Empirically 0x400000 (same)

//  //BLOCK1 __HAL_RCC_TIM6_CLK_ENABLE();
//  __HAL_RCC_TIM6_CLK_ENABLE();

//  //RCC -> APB1ENR for TIM6EN
//  uint32_t* RCC_APB1ENR_OI = (uint32_t *)0x40023840UL; // Empirically 0x40023840 (same)
//  *RCC_APB1ENR_OI |= (uint32_t)(1U << 4U); // Empirically 16 (same)

//  //BLOCK1 HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
//  RCC_ClkInitTypeDef    clkconfig_OI;
//  uint32_t              uwTimclock_OI = 0;
//  uint32_t              uwPrescalerValue_OI = 0;
//  uint32_t              pFLatency_OI;
//  HAL_RCC_GetClockConfig(&clkconfig_OI, &pFLatency_OI); // Changed &clkconfig to &clkconfig_OI, &pFLatency to &pFLatency_OI

//  // HAL_RCC_GetClockConfig()
//  TIM_HandleTypeDef        htim6; // JHM added 2022.11.12 in debug
//  RCC_ClkInitTypeDef    clkconfig_OI;
//  uint32_t              uwTimclock_OI = 0;
//  uint32_t              uwPrescalerValue_OI = 0;
//  uint32_t              pFLatency_OI;
//  // uint32_t* RCC_ClkInitStruct_ClockType_OI = (uint32_t*)0x2004ff9cUL; // Empirically 0x2004ff1c (same) // err maybe actually 0x2004ff1c.. (just wherever RCC_ClkInitStruct->ClockType is)
//  // *RCC_ClkInitStruct_ClockType_OI = 15UL; // Empirically 15 (same) // This eventually changes to 0x40001000, but not sure when or if that's kosher
//  clkconfig_OI->ClockType = 15UL;
//  // uint32_t* RCC_ClkInitStruct_SYSCLKSource_OI = (uint32_t*)0x2004ffa0UL; // Empirically 0x2004ffa0 (same) // err maybe actually 0x2004ff20... (just wherever RCC_ClkInitStruct->SYSCLKSource is)
//  // *RCC_ClkInitStruct_SYSCLKSource_OI = 0UL; // Empirically 0 (same) // This eventually changes to 0x80004AC, but not sure when or if that's kosher
//  clkconfig_OI->SYSCLKSource = 0UL;
//  // uint32_t* RCC_ClkInitStruct_AHBCLKDivider_OI = (uint32_t*)0x2004ffa4UL; // Empirically 0x2004ffa4 (same)// err maybe actually 0x2004ff24... (just wherever RCC_ClkInitStruct->AHBCLKDivider is)
//  // *RCC_ClkInitStruct_AHBCLKDivider_OI = 0UL; // Empirically 0 (same) // Maybe eventually changes but my manual debugging wrote over the before reading
//  clkconfig_OI->AHBCLKDivider = 0UL;
//  // uint32_t* RCC_ClkInitStruct_APB1CLKDivider_OI = (uint32_t*)0x2004ffa8UL; // Empirically 0x2004ffa8 (same) // err maybe actually 0x2004ff28.. (just wherever RCC_ClkInitStruct->APB1CLKDivider is)
//  // *RCC_ClkInitStruct_APB1CLKDivider_OI = 0UL; // Empirically 0 (same)
//  clkconfig_OI->APB1CLKDivider = 0UL;
//  // uint32_t* RCC_ClkInitStruct_APB2CLKDivider_OI = (uint32_t*)0x2004ffacUL; // Empirically 0x2004ffac (same) // err maybe actually 0x2004ff2c.. (just wherever RCC_ClkInitStruct->APB2CLKDivider is)
//  // *RCC_ClkInitStruct_APB2CLKDivider_OI = 0UL; // Empirically 0 (same)
//  clkconfig_OI->APB2CLKDivider = 0UL;
//  // uint32_t* pFLatency_OI = (uint32_t*)0x2004ff98UL; //ALREADY DEFINED // Empirically 0x2004ff98 (same) // err maybe actually 0x2004ff18.. (just wherever FLatency is actually located (thus at pFLatency))
//  // pFLatency_OI = 0UL; // Empirically 0 (same)
//  pFLatency_OI = 0UL; // Empirically 0 (same)
//

//  //BLOCK1 uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
//  uwTimclock_OI = 2*HAL_RCC_GetPCLK1Freq(); // Changed uwTimclock to uwTimclock_OI

//  //Clock Setting
//  uwTimclock_OI = 32000000UL; // Might this need to be "uwTimClock" instead of _OI??

//  //BLOCK1 uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);
//  uwPrescalerValue_OI = (uint32_t) ((uwTimclock_OI / 1000000U) - 1U); // Added _OI 2x

//  uwPrescalerValue_OI = (uint32_t) ((uwTimclock_OI / 1000000U) - 1U);

//  //BLOCK1 htim6.Instance = TIM6; AND next four
//  TIM_HandleTypeDef        htim6; // added but unsure ?? if meets my needs
//  htim6.Instance = TIM6;
//  htim6.Init.Period = (1000000U / 1000U) - 1U;
//  htim6.Init.Prescaler = uwPrescalerValue_OI; // Added OI
//  htim6.Init.ClockDivision = 0;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

//  // uint32_t* htim_Instance_OI = (uint32_t*)0x20000028UL; // Empirically 0x20000028 (same)
//  // *htim_Instance_OI = 0x40001000UL; // Empirically 0x40001000 (same)
//	htim6.Instance = TIM6; // Not sure if/how defined
//  // FOLLOWING NEEDS A REDESIGN AROUND htim (Do I get rid of HAL?)
//  uint32_t* htim6_Init_Period_OI = (uint32_t*)(0x20000028UL + 12U); // Shoot maybe this htim6 reg is at another addy entirely.. // Empirically 0x20000034 (same)
//  // Indeed! I debugged and ran print(&htim6) at this spot and got 0x20000028 + 12, so I swapped out 0x40001000UL
//  *htim6_Init_Period_OI = (uint32_t) ((1000000U / 1000U) - 1U); // Empirically 999 (same)
//  uint32_t* htim6_Init_Prescaler_OI = (uint32_t*)(0x20000028UL + 4U); // Empirically 0x2000002c (same)
//  *htim6_Init_Prescaler_OI = uwPrescalerValue_OI; // Empirically 31 (same??)
//  uint32_t* htim6_Init_ClockDivision_OI = (uint32_t*)(0x20000028UL + 16U); // Empirically 0x20000038 (same)
//  *htim6_Init_ClockDivision_OI = 0; // Empirically 0 (same)
//  uint32_t* htim6_Init_CounterMode_OI = (uint32_t*)(0x20000028UL + 8U); // Empirically 0x20000030 (same)
//  *htim6_Init_CounterMode_OI = 0; // Empirically 0 (same)
//

//  // BLOCK1 HAL_TIM_Base_Init(&htim6)
//  HAL_TIM_Base_Init(&htim6);

//  // 2.4.5 Content Revisited
//
//  //Clock Unlock (offsets *Instance, Init, Channel, *hdma[7])
//  //uint32_t* htim6_Lock_OI = (uint32_t*)(0x40001000UL + 24*4U + 6*4U + 7*1U + (6*4U + 12*4U + 2*1U _+ 6*1U + 10*4U)) // DONE: This just seems wrong, the Lock attribute is not at the TIM6 instance
//  uint32_t* htim6_Lock_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U); // Empirically 0x20000064 (same)
//  *htim6_Lock_OI = (uint32_t)0x0UL; // Empirically 0 (same)
//
//  //Clock State to Busy (offsets *Instance, Init, Channel, *hdma[7], Lock)
//  uint32_t* htim6_State_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U); // Empirically 0x20000065 (same)
//  *htim6_State_OI = 2U; // Empirically 2 (same)
//
//  // Modify_Reg in TIM_Base_SetConfig() doesn't do anything FYI. Same with "TIMx->CR1 = tmpcr1;"
//
//  // htim CR1
//  uint32_t* htim6_CR1_OI = (uint32_t*)0x40001000UL; // Empirically 0x40001000 (same)
//  *htim6_CR1_OI = (uint32_t)1UL; // Empirically 1 (same)
//
//  // htim ARR
//  uint32_t* htim6_ARR_OI = (uint32_t*)0x4000102CUL; // Empirically 0x4000102c (same)
//  *htim6_ARR_OI = (uint32_t)999UL; // Empirically 999 (same)
//
//  // htim PSC
//  uint32_t *htim6_PSC_OI = (uint32_t*)0x40001028UL; // Empirically 0x40001028 (same)
//  *htim6_PSC_OI = (uint32_t)31UL; // Empirically 31 (same)
//
//  // htim EGR
//  uint32_t *htim6_EGR_OI = (uint32_t*)0x40001014UL; // Empirically 0x40001014 (same)
//  *htim6_EGR_OI = (uint32_t)1UL; // Check if this set the following addy to 1 and the above addy stays at 0: 0x40001010UL // Empirically 1 (same) // Looks like this is turned to "1" momentarily to update something, then it's 0, so this implementation seems right
//
//  // Clock DMABurstState
//  uint32_t *htim6_DMABS_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U + 1U + 6*1U + 4*1U); // Empirically 0x20000070 (same)
//  *htim6_DMABS_OI = 1UL; // Empirically 1 (same)
//
//  // htim Channel State
//  uint32_t *htim6_ChSt0_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 0*1U); // Empirically 0x20000066 (same)
//  *htim6_ChSt0_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChSt1_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 1*1U); // Empirically 0x20000067 (same)
//  *htim6_ChSt1_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChSt2_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 2*1U); // Empirically 0x20000068 (same)
//  *htim6_ChSt2_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChSt3_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 3*1U); // Empirically 0x20000069 (same)
//  *htim6_ChSt3_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChSt4_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 4*1U); // Empirically 0x2000006a (same)
//  *htim6_ChSt4_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChSt5_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 5*1U); // Empirically 0x2000006b (same)
//  *htim6_ChSt5_OI = 1UL; // Empirically 1 (same)
//
//  // htim Channel N State
//  uint32_t *htim6_ChNSt0_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 6*1U + 0*1U); // Empirically 0x2000006c (same)
//  *htim6_ChNSt0_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChNSt1_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 6*1U + 1*1U); // Empirically 0x2000006d (same)
//  *htim6_ChNSt1_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChNSt2_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 6*1U + 2*1U); // Empirically 0x2000006e (same)
//  *htim6_ChNSt2_OI = 1UL; // Empirically 1 (same)
//  uint32_t *htim6_ChNSt3_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U +1U + 6*1U + 3*1U); // Empirically 0x2000006f (same)
//  *htim6_ChNSt3_OI = 1UL; // Empirically 1 (same)
//
//  // Clock State to READY (offsets *Instance, Init, Channel, *hdma[7], Lock)
//  // uint32_t *htim6_State_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U); // ALREADY DEFINED // Empirically 0x20000065 (same)
//  *htim6_State_OI = 1; // Empirically 1 (same)
//

//  //BLOCK1 HAL_TIM_Base_Start_IT(&htim6)
//  HAL_TIM_Base_Start_IT(&htim6);

//  //2.4.7
//  // uint32_t *htim6_State_OI = (uint32_t*)(0x20000028UL + 4U + 6*4U + 4U + 7*4U + 1U); // ALREADY DEFINED // Empirically 0x20000065 (same)
//  *htim6_State_OI = 2; // Empirically 2 (same)
//
  // 2.4.7.1
  uint32_t* htim6_Instance_DIER_OI = (uint32_t *)(0x4000100cUL); // Empirically 0x4000100c (same)
  *htim6_Instance_DIER_OI |= (uint32_t)1U; // Empirically 1 (same)

  // 2.4.7.2
  uint32_t* htim_Instance_CR1_OI = (uint32_t *)(0x40001000UL); // Empirically 0x40001000 (same)
  *htim_Instance_CR1_OI |= (uint32_t)1U; // Empirically 1 (same)

//  //BLOCK0 HAL_MspInit();
//  /* Init the low level hardware */
//  HAL_MspInit();

// JHM Code (Working as of 2022.11.12 !)
  // 2.6.1 (__HAL_RCC_PWR_CLK_ENABLE(); --> SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);)
//  uint32_t* RCC_APB1ENR_OI = (uint32_t *)(0x40023840UL); // ALREADY DEFINED // Empirically 0x40023840 (same)
  *RCC_APB1ENR_OI |= (1U << 28U); // Empirically 0x10000000 (same). // WAIT no, empirically it's turned from 10000000 to 0x10000010
  *RCC_APB1ENR_OI |= (1U << 4U); // Factors in the change from above

  // 2.6.2 (__HAL_RCC_SYSCFG_CLK_ENABLE(); --> SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);)
  uint32_t* RCC_APB2ENR_OI = (uint32_t *)(0x40023844UL); // Empirically 0x40023844 (same)
  *RCC_APB2ENR_OI |= (1U << 14U); // Empirically 0x4000 (same)

  // 2.6.3.3
  uint8_t* SCB_SHPR10_OI = (uint8_t *)(0xE000ED22UL); // Empirically 0xe000ed22 (same)
  *SCB_SHPR10_OI = 240UL; // Empirically 240 (same)


  __enable_irq();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//__disable_irq();
//MX_GPIO_Init();
//__enable_irq();

//  MX_ADC3_Init();
//  MX_CRC_Init();
//  MX_DCMI_Init();
//  MX_DMA2D_Init();
//  MX_FMC_Init();
//  MX_I2C1_Init();
//  MX_I2C3_Init();
//  MX_LTDC_Init();
//  MX_QUADSPI_Init();
//  MX_RTC_Init();
//  MX_SAI2_Init();
//  MX_SDMMC1_SD_Init();
//  MX_SPDIFRX_Init();
//  MX_SPI2_Init();
//  MX_TIM1_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  MX_TIM5_Init();
//  MX_TIM8_Init();
//  MX_TIM12_Init();
//  MX_USART1_UART_Init();
//  MX_USART6_UART_Init();
//  MX_USB_HOST_Init();
//  MX_ETH_Init();

  /* USER CODE BEGIN 2 */

  uint32_t* RCC_reg_OI = (uint32_t*)0x40023830UL;
  *RCC_reg_OI |= (0x1UL << 8U);

  uint16_t POS_OI[] = {0x3U, 0x2U, 0xCU};

  for(uint32_t i = 0x0UL; i < (uint32_t)0x3UL; i++)
  {
	  uint16_t temp_POS_OI = POS_OI[i];

	  //SPEED
	  uint32_t* GPIOI_OSPEEDR_OI = (uint32_t*)0x40022008UL;
	  *GPIOI_OSPEEDR_OI &= ~(0x3UL << (temp_POS_OI * 2));

	  //TYPE
	  uint32_t* GPIOI_OTYPER_OI = (uint32_t*)0x40022004UL;
	  *GPIOI_OTYPER_OI &= ~(0x1UL << temp_POS_OI);

	  //PUPD
	  uint32_t* GPIOI_PUPDR_OI = (uint32_t*)0x4002200CUL;
	  *GPIOI_PUPDR_OI &= ~(0x3UL << (temp_POS_OI * 2));

	  //MODE
	  uint32_t* GPIOI_MODER_OI = (uint32_t*)0x40022000UL;
	  *GPIOI_MODER_OI &= ~(0x3UL << (temp_POS_OI * 2));
	  *GPIOI_MODER_OI |= (0x1UL << (temp_POS_OI * 2));

  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Turns out the below block isn't needed, we can just toggle once per loop as I ~knew */
	/*
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_2);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_2);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_2);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_2);
	HAL_Delay(500);
	*/

	/* USER CODE END WHILE */
//    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	// U(S)ART CODE BEGINS HERE ----------------------------------
	__disable_irq();
	// Define SenseAir S8 UART hard variables (per http://www.co2meters.com/Documentation/Datasheets/DS-S8-rev_P11_1_00.pdf)
	uint32_t baud_rate = 9600;

	// Res: This hacking via UART was exactly what I needed to understand start bit (LOW!) and stop bit (HIGH!), I think...
	// https://www.youtube.com/watch?v=01mw0oTHwxg
	// In short, [start=LOW][8 bit message, ex HIGH, LOW, LOW, HIGH, LOW, LOW, HIGH, HIGH][Parity bit probably skipped][Stop=HIGH or HIGH HIGH]
	uint8_t start_bit = 0b0; // Not sure if one or zero. I'm starting to think from [a] and [b] that the start and stop bits are just zeros that don't then need to be written.. [a] http://co2meters.com/Documentation/AppNotes/AN168-S8-raspberry-pi-uart.pdf and [b] https://github.com/jcomas/S8_UART/blob/main/src/s8_uart.cpp
	//uint8_t parity_bit = 0b0; // Parity bit for SenseAir S8 is nonexistent
	uint8_t stop_bits = 0b11; // Per this resource, stop bit for transmission is two bits. Which two? Per above, probably HIGH and... HIGH?

	uint8_t AnyAddress = 0xFE; // Always seems to come after the start bit
	uint8_t FunctionalCode = 0x0;
	uint16_t StartingAddress = 0x0;
	uint16_t QtyOfRegs = 0x0;
	uint16_t CRC = 0x0;

	// Define STM32 "Leader" hard variables
	uint8_t timeDilation = 2; // This is because I didn't set some registers when I ripped out HAL
	uint32_t* GPIO_Port_OI_SenseAirTx_Addy = (uint32_t*)0x40022014UL; // This is actually the reg that's attached to my Oscope in lab. Eventually We'll pick a different pin
	int Pin_Num_OI = 2; // This is the pin on that register
	uint32_t Pin_Reg_OI = 1 << Pin_Num_OI; // bit shifting "1" over to the corresponding pin

    // Determine if needing to check for faults

	// TBD

	// Determine if sending a UART message
	uint8_t initiate_comm = 0;
    if (1) { // TBD
    	initiate_comm = 1;
    }

	// If yes, what UART message to send
	 if (initiate_comm == 1) {
		FunctionalCode = 0x04; // Most common because "Read Input Registers"
		StartingAddress = 0x0003;
		QtyOfRegs = 0x0001;
		CRC = 0xC5D5;
	}

	__enable_irq();
	// If yes, write the actual message
	uint8_t bit_OI = 0;
    if (initiate_comm == 1) {
		int nBytes = 8; // Lengths of AnyAddress + FunctionalCode + StartingAddress + QtyOfReds + CRC
		int nBitsInPacket = 11; // 1-bit Start + 8-bit message + 2-bit stop

		uint8_t fullMessage[8] = {AnyAddress, FunctionalCode, (StartingAddress >> 8), (StartingAddress & 0xFF), (QtyOfRegs >> 8), (QtyOfRegs & 0xFF), (CRC & 0xFF), (CRC >> 8)};
		// above - 8 is from nBytes, but when using nBNytes there was a compile time error (variable-sized object may not be initialized)
		//DEBUG uint8_t fullMessage[8] = { 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0 };

		// As default, main's Tx is HIGH

		for (int aa = 0; aa < nBytes; aa++){
			for (int bb = 0; bb < nBitsInPacket; bb++) {
				// Delay 1/baud since last delay
				JHM_Delay(250);// Temporary while I see if the below logic works

				__disable_irq();
				// Triage logic per spot in the packet
				switch(bb) {
					case 0:
						// Output the start bit to the leader's Tx line (to the follower's Rx)
						JHM_TxPinWrite(GPIO_Port_OI_SenseAirTx_Addy, Pin_Reg_OI, start_bit);
						break;
					case (11 - 2): // 11 from nBitsInPacket
						// Output the first start bit
						JHM_TxPinWrite(GPIO_Port_OI_SenseAirTx_Addy, Pin_Reg_OI, stop_bits >> 1);
						break;
					case (11 - 1): // 11 from nBitsInPacket
						// Output the second start bit
						JHM_TxPinWrite(GPIO_Port_OI_SenseAirTx_Addy, Pin_Reg_OI, stop_bits & 0b1);
						break;
					default:
						// Output the bb'th bit of the aa'th full message (byte)
						bit_OI = ((fullMessage[aa] & (1 << bb)) >> bb); // either result in 0b00000000 or 0b00000001
						JHM_TxPinWrite(GPIO_Port_OI_SenseAirTx_Addy, Pin_Reg_OI, bit_OI);

				}
				__enable_irq();
			}
		}

		// As default, main's Tx is HIGH

    }



	// Determine if trying to receive a UART message

	// If yes, collect each value

    // If yes, then transform the value into a useful.

	// If yes, then display the message somewhere somehow



    __disable_irq();
    /*
    HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_2);
    */

    uint32_t* GPIO_Port_OI_ODR_Addy = (uint32_t*)0x40022014UL;
    // int Pin_Num_OI = 2;
    Pin_Num_OI = 2;
    // uint32_t Pin_Reg_OI = 1 << Pin_Num_OI;
    Pin_Reg_OI = 1 << Pin_Num_OI;
    *GPIO_Port_OI_ODR_Addy = ((*GPIO_Port_OI_ODR_Addy) ^ (Pin_Reg_OI));
    __enable_irq();
	/*HAL_Delay(300);*/
	JHM_Delay(250);

  }
  /* USER CODE END 3 */
}

void JHM_Delay(uint32_t Delay_copy)
{
	uint32_t tickstart_copy = uwTick_copy;
	uint32_t wait_copy = Delay_copy;
	if (wait_copy < 0xFFFFFFFFU)
	{
		wait_copy += (uint32_t)(uwTickFreq_copy);
	}
	while ((uwTick_copy - tickstart_copy) < wait_copy)
	{
		/* Spending a lot of time doing nothing in this loop */
	}
}

void JHM_TxPinWrite(uint32_t* GPIO_Port_OI_Addy, uint32_t PinWithinPort, uint8_t bit2write)
{
	if (bit2write) { // If non-zero, so we'd assume the bit is "1"
		*GPIO_Port_OI_Addy = *GPIO_Port_OI_Addy | PinWithinPort;
	}
	else { // If zero "0"
		*GPIO_Port_OI_Addy = *GPIO_Port_OI_Addy & ~PinWithinPort;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//  /** Configure LSE Drive Capability
//  */
//  HAL_PWR_EnableBkUpAccess();
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 25;
//  RCC_OscInitStruct.PLL.PLLN = 400;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Activate the Over-Drive mode
//  */
//  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX|RCC_PERIPHCLK_LTDC
//                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
//                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_SAI2
//                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
//                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
//  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
//  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
//  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
//  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
//  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
//  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
//  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
//  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
//  PeriphClkInitStruct.PLLI2SDivQ = 1;
//  PeriphClkInitStruct.PLLSAIDivQ = 1;
//  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
//  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
//  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
//  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
//  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
//  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
//  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
//  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
//  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC3_Init(void)
//{
//
//  /* USER CODE BEGIN ADC3_Init 0 */
//
//  /* USER CODE END ADC3_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC3_Init 1 */
//
//  /* USER CODE END ADC3_Init 1 */
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//  hadc3.Instance = ADC3;
//  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc3.Init.ContinuousConvMode = DISABLE;
//  hadc3.Init.DiscontinuousConvMode = DISABLE;
//  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc3.Init.NbrOfConversion = 1;
//  hadc3.Init.DMAContinuousRequests = DISABLE;
//  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  if (HAL_ADC_Init(&hadc3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_7;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC3_Init 2 */
//
//  /* USER CODE END ADC3_Init 2 */
//
//}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_CRC_Init(void)
//{
//
//  /* USER CODE BEGIN CRC_Init 0 */
//
//  /* USER CODE END CRC_Init 0 */
//
//  /* USER CODE BEGIN CRC_Init 1 */
//
//  /* USER CODE END CRC_Init 1 */
//  hcrc.Instance = CRC;
//  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
//  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
//  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
//  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
//  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
//  if (HAL_CRC_Init(&hcrc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN CRC_Init 2 */
//
//  /* USER CODE END CRC_Init 2 */
//
//}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
//static void MX_DCMI_Init(void)
//{
//
//  /* USER CODE BEGIN DCMI_Init 0 */
//
//  /* USER CODE END DCMI_Init 0 */
//
//  /* USER CODE BEGIN DCMI_Init 1 */
//
//  /* USER CODE END DCMI_Init 1 */
//  hdcmi.Instance = DCMI;
//  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
//  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
//  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
//  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
//  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
//  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
//  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
//  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
//  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
//  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
//  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
//  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DCMI_Init 2 */
//
//  /* USER CODE END DCMI_Init 2 */
//
//}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
//static void MX_DMA2D_Init(void)
//{
//
//  /* USER CODE BEGIN DMA2D_Init 0 */
//
//  /* USER CODE END DMA2D_Init 0 */
//
//  /* USER CODE BEGIN DMA2D_Init 1 */
//
//  /* USER CODE END DMA2D_Init 1 */
//  hdma2d.Instance = DMA2D;
//  hdma2d.Init.Mode = DMA2D_M2M;
//  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
//  hdma2d.Init.OutputOffset = 0;
//  hdma2d.LayerCfg[1].InputOffset = 0;
//  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
//  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
//  hdma2d.LayerCfg[1].InputAlpha = 0;
//  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DMA2D_Init 2 */
//
//  /* USER CODE END DMA2D_Init 2 */
//
//}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ETH_Init(void)
//{
//
//  /* USER CODE BEGIN ETH_Init 0 */
//
//  /* USER CODE END ETH_Init 0 */
//
//   static uint8_t MACAddr[6];
//
//  /* USER CODE BEGIN ETH_Init 1 */
//
//  /* USER CODE END ETH_Init 1 */
//  heth.Instance = ETH;
//  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
//  heth.Init.Speed = ETH_SPEED_100M;
//  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
//  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
//  MACAddr[0] = 0x00;
//  MACAddr[1] = 0x80;
//  MACAddr[2] = 0xE1;
//  MACAddr[3] = 0x00;
//  MACAddr[4] = 0x00;
//  MACAddr[5] = 0x00;
//  heth.Init.MACAddr = &MACAddr[0];
//  heth.Init.RxMode = ETH_RXPOLLING_MODE;
//  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
//  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
//
//  /* USER CODE BEGIN MACADDRESS */
//
//  /* USER CODE END MACADDRESS */
//
//  if (HAL_ETH_Init(&heth) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ETH_Init 2 */
//
//  /* USER CODE END ETH_Init 2 */
//
//}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
//
//  /* USER CODE END I2C1_Init 0 */
//
//  /* USER CODE BEGIN I2C1_Init 1 */
//
//  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.Timing = 0x00C0EAFF;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Analogue filter
//  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Digital filter
//  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
//
//  /* USER CODE END I2C1_Init 2 */
//
//}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_I2C3_Init(void)
//{
//
//  /* USER CODE BEGIN I2C3_Init 0 */
//
//  /* USER CODE END I2C3_Init 0 */
//
//  /* USER CODE BEGIN I2C3_Init 1 */
//
//  /* USER CODE END I2C3_Init 1 */
//  hi2c3.Instance = I2C3;
//  hi2c3.Init.Timing = 0x00C0EAFF;
//  hi2c3.Init.OwnAddress1 = 0;
//  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c3.Init.OwnAddress2 = 0;
//  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Analogue filter
//  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Digital filter
//  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C3_Init 2 */
//
//  /* USER CODE END I2C3_Init 2 */
//
//}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_LTDC_Init(void)
//{
//
//  /* USER CODE BEGIN LTDC_Init 0 */
//
//  /* USER CODE END LTDC_Init 0 */
//
//  LTDC_LayerCfgTypeDef pLayerCfg = {0};
//
//  /* USER CODE BEGIN LTDC_Init 1 */
//
//  /* USER CODE END LTDC_Init 1 */
//  hltdc.Instance = LTDC;
//  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
//  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
//  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
//  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
//  hltdc.Init.HorizontalSync = 40;
//  hltdc.Init.VerticalSync = 9;
//  hltdc.Init.AccumulatedHBP = 53;
//  hltdc.Init.AccumulatedVBP = 11;
//  hltdc.Init.AccumulatedActiveW = 533;
//  hltdc.Init.AccumulatedActiveH = 283;
//  hltdc.Init.TotalWidth = 565;
//  hltdc.Init.TotalHeigh = 285;
//  hltdc.Init.Backcolor.Blue = 0;
//  hltdc.Init.Backcolor.Green = 0;
//  hltdc.Init.Backcolor.Red = 0;
//  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pLayerCfg.WindowX0 = 0;
//  pLayerCfg.WindowX1 = 480;
//  pLayerCfg.WindowY0 = 0;
//  pLayerCfg.WindowY1 = 272;
//  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
//  pLayerCfg.Alpha = 255;
//  pLayerCfg.Alpha0 = 0;
//  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
//  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
//  pLayerCfg.FBStartAdress = 0xC0000000;
//  pLayerCfg.ImageWidth = 480;
//  pLayerCfg.ImageHeight = 272;
//  pLayerCfg.Backcolor.Blue = 0;
//  pLayerCfg.Backcolor.Green = 0;
//  pLayerCfg.Backcolor.Red = 0;
//  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN LTDC_Init 2 */
//
//  /* USER CODE END LTDC_Init 2 */
//
//}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
//static void MX_QUADSPI_Init(void)
//{
//
//  /* USER CODE BEGIN QUADSPI_Init 0 */
//
//  /* USER CODE END QUADSPI_Init 0 */
//
//  /* USER CODE BEGIN QUADSPI_Init 1 */
//
//  /* USER CODE END QUADSPI_Init 1 */
//  /* QUADSPI parameter configuration*/
//  hqspi.Instance = QUADSPI;
//  hqspi.Init.ClockPrescaler = 1;
//  hqspi.Init.FifoThreshold = 4;
//  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
//  hqspi.Init.FlashSize = 24;
//  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
//  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
//  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
//  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
//  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN QUADSPI_Init 2 */
//
//  /* USER CODE END QUADSPI_Init 2 */
//
//}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};
//  RTC_AlarmTypeDef sAlarm = {0};
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = 127;
//  hrtc.Init.SynchPrediv = 255;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* USER CODE BEGIN Check_RTC_BKUP */
//
//  /* USER CODE END Check_RTC_BKUP */
//
//  /** Initialize RTC and set the Time and Date
//  */
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 0x1;
//  sDate.Year = 0x0;
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Enable the Alarm A
//  */
//  sAlarm.AlarmTime.Hours = 0x0;
//  sAlarm.AlarmTime.Minutes = 0x0;
//  sAlarm.AlarmTime.Seconds = 0x0;
//  sAlarm.AlarmTime.SubSeconds = 0x0;
//  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
//  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
//  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//  sAlarm.AlarmDateWeekDay = 0x1;
//  sAlarm.Alarm = RTC_ALARM_A;
//  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Enable the Alarm B
//  */
//  sAlarm.Alarm = RTC_ALARM_B;
//  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Enable the TimeStamp
//  */
//  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_POS1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SAI2_Init(void)
//{
//
//  /* USER CODE BEGIN SAI2_Init 0 */
//
//  /* USER CODE END SAI2_Init 0 */
//
//  /* USER CODE BEGIN SAI2_Init 1 */
//
//  /* USER CODE END SAI2_Init 1 */
//  hsai_BlockA2.Instance = SAI2_Block_A;
//  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
//  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
//  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
//  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
//  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
//  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
//  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
//  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
//  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
//  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
//  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
//  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
//  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
//  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
//  hsai_BlockA2.FrameInit.FrameLength = 8;
//  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
//  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
//  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
//  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
//  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
//  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
//  hsai_BlockA2.SlotInit.SlotNumber = 1;
//  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
//  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  hsai_BlockB2.Instance = SAI2_Block_B;
//  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
//  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
//  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
//  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
//  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
//  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
//  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
//  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
//  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
//  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
//  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
//  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
//  hsai_BlockB2.FrameInit.FrameLength = 8;
//  hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
//  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
//  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
//  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
//  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
//  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
//  hsai_BlockB2.SlotInit.SlotNumber = 1;
//  hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
//  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SAI2_Init 2 */
//
//  /* USER CODE END SAI2_Init 2 */
//
//}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */

/*static void MX_SDMMC1_SD_Init(void)
{

   USER CODE BEGIN SDMMC1_Init 0

   USER CODE END SDMMC1_Init 0

   USER CODE BEGIN SDMMC1_Init 1

   USER CODE END SDMMC1_Init 1
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
   USER CODE BEGIN SDMMC1_Init 2

   USER CODE END SDMMC1_Init 2

}*/


/**
  * @brief SPDIFRX Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPDIFRX_Init(void)
//{
//
//  /* USER CODE BEGIN SPDIFRX_Init 0 */
//
//  /* USER CODE END SPDIFRX_Init 0 */
//
//  /* USER CODE BEGIN SPDIFRX_Init 1 */
//
//  /* USER CODE END SPDIFRX_Init 1 */
//  hspdif.Instance = SPDIFRX;
//  hspdif.Init.InputSelection = SPDIFRX_INPUT_IN0;
//  hspdif.Init.Retries = SPDIFRX_MAXRETRIES_NONE;
//  hspdif.Init.WaitForActivity = SPDIFRX_WAITFORACTIVITY_OFF;
//  hspdif.Init.ChannelSelection = SPDIFRX_CHANNEL_A;
//  hspdif.Init.DataFormat = SPDIFRX_DATAFORMAT_LSB;
//  hspdif.Init.StereoMode = SPDIFRX_STEREOMODE_DISABLE;
//  hspdif.Init.PreambleTypeMask = SPDIFRX_PREAMBLETYPEMASK_OFF;
//  hspdif.Init.ChannelStatusMask = SPDIFRX_CHANNELSTATUS_OFF;
//  hspdif.Init.ValidityBitMask = SPDIFRX_VALIDITYMASK_OFF;
//  hspdif.Init.ParityErrorMask = SPDIFRX_PARITYERRORMASK_OFF;
//  if (HAL_SPDIFRX_Init(&hspdif) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPDIFRX_Init 2 */
//
//  /* USER CODE END SPDIFRX_Init 2 */
//
//}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
//
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
//
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_MASTER;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 7;
//  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
//
//  /* USER CODE END SPI2_Init 2 */
//
//}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 65535;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//  sBreakDeadTimeConfig.Break2Filter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//  HAL_TIM_MspPostInit(&htim1);
//
//}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 0;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 4294967295;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//  HAL_TIM_MspPostInit(&htim2);
//
//}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM3_Init(void)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 0;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 65535;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(&htim3);
//
//}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM5_Init(void)
//{
//
//  /* USER CODE BEGIN TIM5_Init 0 */
//
//  /* USER CODE END TIM5_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM5_Init 1 */
//
//  /* USER CODE END TIM5_Init 1 */
//  htim5.Instance = TIM5;
//  htim5.Init.Prescaler = 0;
//  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim5.Init.Period = 4294967295;
//  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM5_Init 2 */
//
//  /* USER CODE END TIM5_Init 2 */
//  HAL_TIM_MspPostInit(&htim5);
//
//}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM8_Init(void)
//{
//
//  /* USER CODE BEGIN TIM8_Init 0 */
//
//  /* USER CODE END TIM8_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM8_Init 1 */
//
//  /* USER CODE END TIM8_Init 1 */
//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = 0;
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim8.Init.Period = 65535;
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM8_Init 2 */
//
//  /* USER CODE END TIM8_Init 2 */
//
//}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM12_Init(void)
//{
//
//  /* USER CODE BEGIN TIM12_Init 0 */
//
//  /* USER CODE END TIM12_Init 0 */
//
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM12_Init 1 */
//
//  /* USER CODE END TIM12_Init 1 */
//  htim12.Instance = TIM12;
//  htim12.Init.Prescaler = 0;
//  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim12.Init.Period = 65535;
//  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM12_Init 2 */
//
//  /* USER CODE END TIM12_Init 2 */
//  HAL_TIM_MspPostInit(&htim12);
//
//}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART6_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART6_Init 0 */
//
//  /* USER CODE END USART6_Init 0 */
//
//  /* USER CODE BEGIN USART6_Init 1 */
//
//  /* USER CODE END USART6_Init 1 */
//  huart6.Instance = USART6;
//  huart6.Init.BaudRate = 115200;
//  huart6.Init.WordLength = UART_WORDLENGTH_8B;
//  huart6.Init.StopBits = UART_STOPBITS_1;
//  huart6.Init.Parity = UART_PARITY_NONE;
//  huart6.Init.Mode = UART_MODE_TX_RX;
//  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART6_Init 2 */
//
//  /* USER CODE END USART6_Init 2 */
//
//}

/* FMC initialization function */
//static void MX_FMC_Init(void)
//{
//
//  /* USER CODE BEGIN FMC_Init 0 */
//
//  /* USER CODE END FMC_Init 0 */
//
//  FMC_SDRAM_TimingTypeDef SdramTiming = {0};
//
//  /* USER CODE BEGIN FMC_Init 1 */
//
//  /* USER CODE END FMC_Init 1 */
//
//  /** Perform the SDRAM1 memory initialization sequence
//  */
//  hsdram1.Instance = FMC_SDRAM_DEVICE;
//  /* hsdram1.Init */
//  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
//  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
//  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
//  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
//  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
//  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
//  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
//  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
//  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
//  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
//  /* SdramTiming */
//  SdramTiming.LoadToActiveDelay = 2;
//  SdramTiming.ExitSelfRefreshDelay = 7;
//  SdramTiming.SelfRefreshTime = 4;
//  SdramTiming.RowCycleDelay = 7;
//  SdramTiming.WriteRecoveryTime = 3;
//  SdramTiming.RPDelay = 2;
//  SdramTiming.RCDDelay = 2;
//
//  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
//  {
//    Error_Handler( );
//  }
//
//  /* USER CODE BEGIN FMC_Init 2 */
//
//  /* USER CODE END FMC_Init 2 */
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void) // 2022.11.13, I thought I needed this function - or at least the part that wasn't commented out. But I cut the call of this function and correctly figured I could also comment out this definitions
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
////  __HAL_RCC_GPIOE_CLK_ENABLE();
////  __HAL_RCC_GPIOG_CLK_ENABLE();
////  __HAL_RCC_GPIOB_CLK_ENABLE();
////  __HAL_RCC_GPIOD_CLK_ENABLE();
////  __HAL_RCC_GPIOC_CLK_ENABLE();
////  __HAL_RCC_GPIOA_CLK_ENABLE();
////  __HAL_RCC_GPIOJ_CLK_ENABLE();
//  __HAL_RCC_GPIOI_CLK_ENABLE();
////  __HAL_RCC_GPIOK_CLK_ENABLE();
////  __HAL_RCC_GPIOF_CLK_ENABLE();
////  __HAL_RCC_GPIOH_CLK_ENABLE();
//
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);
//
//  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin, GPIO_PIN_RESET);
//
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);
////
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);
////
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);
////
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);
//
////  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
////  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
////  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : Audio_INT_Pin */
////  GPIO_InitStruct.Pin = Audio_INT_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
////  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
//  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
//
////  /*Configure GPIO pin : uSD_Detect_Pin */
////  GPIO_InitStruct.Pin = uSD_Detect_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
////  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
////  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pins : TP3_Pin NC2_Pin */
////  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
////  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : LCD_INT_Pin */
////  GPIO_InitStruct.Pin = LCD_INT_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
////  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : RMII_RXER_Pin */
////  GPIO_InitStruct.Pin = RMII_RXER_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);
////
////  /*Configure GPIO pin : ULPI_D3_Pin */
////  GPIO_InitStruct.Pin = ULPI_D3_Pin;
////  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
////  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
////  HAL_GPIO_Init(ULPI_D3_GPIO_Port, &GPIO_InitStruct);
//
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM6) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//
//  /* USER CODE END Callback 1 */
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
