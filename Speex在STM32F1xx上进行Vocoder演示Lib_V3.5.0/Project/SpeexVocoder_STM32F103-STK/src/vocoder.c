/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/vocoder.c 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    This file provides all the vocoder firmware functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Includes ------------------------------------------------------------------*/
#include "vocoder.h"


/** @addtogroup SpeexVocoder_STM32F103_STK
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Initializes Voice recording/playing
  * @param  None
  * @retval : None.
  */
void Vocoder_Init(void)
{
  /* Peripherals InitStructure define -----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* ADC configuration --------------------------------------------------------*/
  
  /* ADC Channel 1 pin configuration */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
     
  /* ADC1 structre initialization */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel1 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_13Cycles5);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* TIM1 configuration -------------------------------------------------------*/
  TIM_DeInit(TIM1);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  GPIO_StructInit(&GPIO_InitStructure);

  /* Configure PA.08 as alternate function (TIM1_OC1) */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* TIM1 used for PWM genration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x00; /* TIM1CLK = 72 MHz */
  TIM_TimeBaseStructure.TIM_Period = 0x3FF; /* 10 bits resolution */
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  /* TIM1's Channel1 in PWM1 mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x200;/* Duty cycle: 50%*/
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;  
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM1, ENABLE); 

  /* TIM2 configuration -------------------------------------------------------*/
  TIM_DeInit(TIM2);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  /* TIM2 used for timing, the timing period depends on the sample rate */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x00;    /* TIM2CLK = 72 MHz */
  TIM_TimeBaseStructure.TIM_Period = TIM2ARRValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* Output Compare Inactive Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  TIM_OCInitStructure.TIM_Pulse = 0x0;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  /* TIM3 configuration -------------------------------------------------------*/
  
  TIM_DeInit(TIM3);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  /* TIM3 used for timing, the timing period depends on the sample rate */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x00;    /* TIM3CLK = 72 MHz */
  TIM_TimeBaseStructure.TIM_Period = TIM3ARRValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* Output Compare Inactive Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  TIM_OCInitStructure.TIM_Pulse = 0x0;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Only for OLIMEX STM32F103-STK board --------------------------------------*/
  /* Set STNBY pin of the TS4871 amplifier on the board STM32F103-STK from OLIMEX */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/**
  * @brief  Start playing
  * @param  None
  * @retval : None
  */
void Vocoder_Start(void)
{
  /* ADC1 regular Software Start Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  /* Relaod ARR register */
  TIM2->ARR = TIM2ARRValue;
  
  /* Enable the TIM Counter */
  TIM2->CR1 |= CR1_CEN_Set;

  /* Clear the IT pending Bit */
  TIM2->SR = (uint16_t)TIM_INT_Update;

  /* Enable TIM2 update interrupt */
  TIM2->DIER |= TIM_IT_Update;

  /* Only for OLIMEX STM32F103-STK board --------------------------------------*/
  /* TS4871 on */
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_RESET);

}



/**
  * @brief  Stop vocoder 
  * @param  None
  * @retval : None
  */
void Vocoder_Stop(void)
{
  /* ADC1 regular Software Stop Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, DISABLE);
  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);

  /* Stop TIM2 */
  TIM_Cmd(TIM2, DISABLE);

  /* Disable TIM2 update interrupt */
  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

  /* Only for OLIMEX STM32F103-STK board --------------------------------------*/
  /* TS4871 off */
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_SET);

}



/**
  * @brief  Erases the FLASH recording area
  * @param  None
  * @retval : None
  */
void Voice_Recording_Init(void)
{
  uint32_t page_nbr;
  FLASH_Status FLASHStatus = FLASH_COMPLETE;

  /* Flash unlock */
  FLASH_Unlock();

  /* Erase the needed pages where the user recorded voice will be loaded */
  for(page_nbr = 0; (page_nbr < ALL_PAGES) && (FLASHStatus == FLASH_COMPLETE); page_nbr++)
  {
     FLASHStatus = FLASH_ErasePage(RECORDING_START_ADDRESS + (PAGE_SIZE * page_nbr));
  }

}



/**
  * @brief  Start voice recording
  * @param  None
  * @retval : None
  */
void Voice_Recording_Start(void)
{
  /* TS4871 off */
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_SET);

  /* ADC1 regular Software Start Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Relaod ARR register */
  TIM3->ARR = TIM3ARRValue;
  
  /* Enable the TIM Counter */
  TIM3->CR1 |= CR1_CEN_Set;

  /* Clear the IT pending Bit */
  TIM3->SR = (uint16_t)TIM_INT_Update;

  /* Enable TIM3 update interrupt */
  TIM3->DIER |= TIM_IT_Update;
  
}



/**
  * @brief  Start voice playing
  * @param  None
  * @retval : None
  */
void Voice_Playing_Start(void)
{  
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  
  /* Relaod ARR register */
  TIM3->ARR = TIM3ARRValue;
  
  /* Enable the TIM Counter */
  TIM3->CR1 |= CR1_CEN_Set;

  /* Clear the IT pending Bit */
  TIM3->SR = (uint16_t)TIM_INT_Update;

  /* Enable TIM3 update interrupt */
  TIM3->DIER |= TIM_IT_Update;
  
  /* Only for OLIMEX STM32F103-STK board --------------------------------------*/
  /* TS4871 on */
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_RESET);
}



/**
  * @brief  Stop voice recording
  * @param  None
  * @retval : None
  */
void Voice_Recording_Stop(void)
{
  /* ADC1 regular Software Stop Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  /* Stop TIM3 */
  TIM_Cmd(TIM3, DISABLE);

  /* Clear the IT pending Bit */
  TIM3->SR = (uint16_t)TIM_INT_Update;

  /* Disable TIM3 update interrupt */
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

  /* Only for OLIMEX STM32F103-STK board --------------------------------------*/
  /* TS4871 off */
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_SET);
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
