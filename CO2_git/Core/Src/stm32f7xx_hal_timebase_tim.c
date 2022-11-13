/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_hal_timebase_TIM.c
  * @brief   HAL time base based on the hardware TIM.
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim6;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM6 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) // JHM added the last input
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  /*Configure the TIM6 IRQ priority */
  // HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority ,0); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Enable the TIM6 global Interrupt */
  // HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Enable TIM6 clock */
  // __HAL_RCC_TIM6_CLK_ENABLE(); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Get clock configuration */
  // HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Compute TIM6 clock */
  uwTimclock = 32000000UL;// JHM changed from "2*HAL_RCC_GetPCLK1Freq()" to "32000000UL" to see if the functional call could be replaced and it succeeded!
  /* Compute the prescaler value to have TIM6 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Initialize TIM6 */
  htim6.Instance = TIM6;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim6.Init.Period = (1000000U / 1000U) - 1U;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
  htim6.Init.Prescaler = uwPrescalerValue;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
  htim6.Init.ClockDivision = 0;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  if(HAL_TIM_Base_Init(&htim6) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim6);
  }

  /* Return function status */
  return HAL_ERROR;
}

HAL_StatusTypeDef HAL_InitTick_JHM(uint32_t TickPriority) // JHM added the last input
{
//  RCC_ClkInitTypeDef    clkconfig;
//  uint32_t              uwTimclock = 0;
//  uint32_t              uwPrescalerValue = 0;
//  uint32_t              pFLatency;
//  /*Configure the TIM6 IRQ priority */
//  // HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority ,0); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//
//  /* Enable the TIM6 global Interrupt */
//  // HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//
//  /* Enable TIM6 clock */
//  // __HAL_RCC_TIM6_CLK_ENABLE(); // JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//
//  /* Get clock configuration */
//  // HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//
//  /* Compute TIM6 clock */
//  uwTimclock = 32000000UL;// JHM changed from "2*HAL_RCC_GetPCLK1Freq()" to "32000000UL" to see if the functional call could be replaced and it succeeded!
//  /* Compute the prescaler value to have TIM6 counter clock equal to 1MHz */
//  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  /* Initialize TIM6 */
//  htim6.Instance = TIM6;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

  uint32_t* htim_Instance_OI = (uint32_t*)0x20000028UL; // Empirically 0x20000028 (same)
  *htim_Instance_OI = 0x40001000UL; // Empirically 0x40001000 (same)

  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
//  htim6.Init.Period = (1000000U / 1000U) - 1U;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//  htim6.Init.Prescaler = 31;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//  htim6.Init.ClockDivision = 0;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;// JHM commented out  to see if I could pull this out of the function itself (and put in main.c and eventually distill down to my code)

//  if(HAL_TIM_Base_Init(&htim6) == HAL_OK)
//  {
//    /* Start the TIM time Base generation in interrupt mode */
//    return HAL_TIM_Base_Start_IT(&htim6);
//  }
//
//  /* Return function status */
//  return HAL_ERROR;

//  HAL_TIM_Base_Init(&htim6); // TODO: Try replacing with the CR1 through EGR updates

    // JHM: HAHAHAHAHA if I just omit this, it works fine

//  HAL_TIM_Base_Start_IT(&htim6);

//    // JHM: HELL YEAH, this as a replacement for HAL_TIM_Base_Start_IT(&htim6) is working swimmingly
//    // 2.4.7.1
//	uint32_t* htim6_Instance_DIER_OI = (uint32_t *)(0x4000100cUL); // Empirically 0x4000100c (same)
//	*htim6_Instance_DIER_OI |= (uint32_t)1U; // Empirically 1 (same)
//
//	// 2.4.7.2
//	uint32_t* htim_Instance_CR1_OI = (uint32_t *)(0x40001000UL); // Empirically 0x40001000 (same)
//	*htim_Instance_CR1_OI |= (uint32_t)1U; // Empirically 1 (same)


}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM6 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM6 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
