/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/stm32f10x_it.c 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Main Interrupt Service Routines.
  *           This file provides template for all exceptions handler and 
  *           peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "main.h"

/** @addtogroup SpeexVocoder_STM32F103_STK
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define EXTTRIG_SWSTART      (u32)(0x00500000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO int16_t IN_Buffer[2][160], OUT_Buffer[2][160];
extern __IO uint8_t Start_Playing;
extern __IO uint8_t Recording;
extern __IO uint8_t Playing;
extern __IO uint8_t Start_Encoding;
extern __IO uint8_t Start_Decoding;
extern uint32_t Encoded_Frames;
extern __IO int16_t *inBuffer;
extern __IO int16_t *outBuffer;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
  static uint8_t spin=0;

  /* Relaod output compare */
  TIM2->ARR = TIM2ARRValue;
  
  /* Clear TIM2 update interrupt */
  TIM2->SR = TIM_INT_Update;

  spin=spin^1;

  if(spin)
  {  
    TIM1->CCR1 = ((*outBuffer>>6)) + 0x200 ;
 
    if(outBuffer == &OUT_Buffer[1][159])
    {
      outBuffer = OUT_Buffer[0];
      Start_Decoding = 2;
    }
    else if(outBuffer == &OUT_Buffer[0][159])
    {
      outBuffer++;
      Start_Decoding = 1;
    }
    else
    {
      outBuffer++;
    }
  }
  else
  {
    if(Start_Playing==REAL_VOICE)
    {
      *inBuffer = (ADC1->DR)^0x8000;
      ADC1->CR2 |= (EXTTRIG_SWSTART);
    }
    if(inBuffer == &IN_Buffer[1][159])
    {
      Start_Encoding = 2;
      inBuffer = IN_Buffer[0];	 
    }
    else if(inBuffer == &IN_Buffer[0][159])
    {
      Start_Encoding = 1;
      inBuffer++;
    }
    else
    {
      inBuffer++;
    }
  } 
}


/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM3_IRQHandler(void)
{
 /* Relaod output compare */
  TIM3->ARR = TIM3ARRValue;
  
  /* Clear TIM2 update interrupt */
  TIM3->SR = TIM_INT_Update;
 
  if(Recording)
  {
    *inBuffer = ((ADC1->DR)^0x8000);
    ADC1->CR2 |= (EXTTRIG_SWSTART);
    
    if(inBuffer == &IN_Buffer[1][159])
    {
      Start_Encoding = 2;
      inBuffer = IN_Buffer[0];
      Encoded_Frames++; 
    }
    else if(inBuffer == &IN_Buffer[0][159])
    {
      Start_Encoding = 1;
      Encoded_Frames++;
      inBuffer++;
    }
    else
    {
      inBuffer++;
    }
    
    if(Encoded_Frames == _1MIN_NB_OF_FRAMES)
    {
      Voice_Recording_Stop();
      Recording=0;
    } 
  }
  else if(Playing) 
  {
    TIM1->CCR1 = ((*outBuffer>>6)) + 0x200 ;
 
    if(outBuffer == &OUT_Buffer[0][159])
    {
      Start_Decoding = 1;
      outBuffer++;
    }
    else if(outBuffer == &OUT_Buffer[1][159])
    {
      outBuffer = OUT_Buffer[0];
      Start_Decoding = 2;
    }
    else
    {
      outBuffer++;
    }	 
  } 
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (SpeexVocoder_OLIMEX_src), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles SpeexVocoder_OLIMEX_src interrupt request.
  * @param  None
  * @retval : None
  */
/*void SpeexVocoder_OLIMEX_src_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
