/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/main.c 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Main program body
  ******************************************************************************
  * @copy
  *	主意修改启动文件里：	Heap_Size    EQU     0x00002000; 8KB 以免出现硬件异常
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
#include "sys.h"
#include "main.h"
#include "voice.h"
#include "lcd.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <speex/speex.h>
#include"arch.h"

/** @addtogroup SpeexVocoder_STM32F103_STK
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ALL_FRAMES      300   /* the encoded male voice length */

#define LCD_DELAY       0x7FFFFF
#define JOYS_DELAY      0x10FFFF

#define KEY_NONE        0
#define KEY_UP          1
#define KEY_DOWN        2
#define KEY_LEFT        3
#define KEY_RIGHT       4
#define KEY_CENTER      5

#define UP_VALUE        960
#define DOWN_VALUE      190
#define LEFT_VALUE      1990
#define RIGHT_VALUE     470
#define DIVERSION       30

#define MENU_NUMB       5
#define MENU_WIDTH      15

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO int16_t IN_Buffer[2][FRAME_SIZE];
__IO int16_t OUT_Buffer[2][FRAME_SIZE];
__IO int16_t *inBuffer = IN_Buffer[0];
__IO int16_t *outBuffer = OUT_Buffer[0];
uint32_t Flash_Address = RECORDING_START_ADDRESS;
__IO uint8_t Start_Encoding = 0;
__IO uint8_t Start_Decoding=0;
__IO uint8_t Start_Playing = 0;
__IO uint8_t Recording = 0;
__IO uint8_t Playing =0;
uint32_t Encoded_Frames=0;

SpeexBits bits;/* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */

char out_bytes[ENCODED_FRAME_SIZE];
char input_bytes[ENCODED_FRAME_SIZE];
uint16_t sample_index = 0;
__IO uint16_t NB_Frames=0;
uint8_t *pFlash = (uint8_t*)RECORDING_START_ADDRESS;

uint8_t Menu[MENU_NUMB][MENU_WIDTH] = {
 "  ** Menu **  \0",
 "Stored SPX msg\0",
 "Real time play\0",
 "Record voice  \0",
 "Play rec.     \0",
};

uint8_t JoyPos;
uint8_t MenuPos;
__IO uint32_t dly;
uint16_t ADCValue;

/* Private function prototypes -----------------------------------------------*/
void Demo_Init(void);
void Speex_Init(void);
void InterruptConfig(void);
void InitJoystick(void);
void UpdateMenu(unsigned char pos);
uint8_t GetJoystickPosition (void);

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{

  Demo_Init();
  Speex_Init();
    
  /* LCD init */
  LCDInit();
  LCDContrast(0x45);
  
  /* Display welcome message */
  LCDClear();
  LCDStr ( 0, "**************", 0 );
  LCDStr ( 1, "  Welcome to  ", 0 );
  LCDStr ( 2, "   STM32F10x  ", 0 );
  LCDStr ( 3, " vocoder demo ", 0 );
  LCDStr ( 4, "by Speex codec", 0 );
  LCDStr ( 5, "**************", 0 );
  LCDUpdate();

  /* Delay */
  for(dly=0; dly<LCD_DELAY; dly++);

MENU_LABEL:

  /* Init joystick */
  InitJoystick();
  
  /* Draw menu */
  UpdateMenu(MenuPos);

  while(1) 
  {
    JoyPos = GetJoystickPosition();

    if(JoyPos==KEY_UP)
    {
      MenuPos--;
      if((MenuPos<1)) MenuPos = 1;
      UpdateMenu(MenuPos);
      for(dly=0; dly<JOYS_DELAY; dly++);
    }

    if(JoyPos==KEY_DOWN)
    {
      MenuPos++;
      if((MenuPos>4)) MenuPos = 4;
      UpdateMenu(MenuPos);
      for(dly=0; dly<JOYS_DELAY; dly++);
    }

    if(JoyPos==KEY_CENTER) break;
  }

  /* Playing welcome message ---------------------------------------------------*/
  if(MenuPos == 1) 
  {
    int i;
    LCDClear();
    LCDStr ( 0, "Stored encoded", 0 );
    LCDStr ( 1, "  male voice  ", 0 );
    LCDStr ( 2, "**************", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();

    Start_Playing = MALE_VOICE;
    Vocoder_Init();
  
    /* we prepare two buffers of decoded data: */
    /* the first one, */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = male_voice[sample_index++];
    } 
      
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
  
    /* and the second one. */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = male_voice[sample_index++];
    } 
 
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);

    NB_Frames++;

    Vocoder_Start();
  
    /* Now we wait until the playing of the buffers to re-decode ...*/
    while((NB_Frames < ALL_FRAMES))//&&(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    {
      if(Start_Decoding == 1) /* we start decoding the first buffer */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* Copy the encoded data into the bit-stream struct */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
      if(Start_Decoding == 2) /* we start decoding the second buffer */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* Copy the encoded data into the bit-stream struct */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
    }
    
    sample_index = 0;
    NB_Frames = 0;
    inBuffer = IN_Buffer[0];
    outBuffer = OUT_Buffer[0];
    Vocoder_Stop();
    goto MENU_LABEL;
  }

  /* Real time playing ---------------------------------------------------------*/
  if(MenuPos == 2) 
  {  
    LCDClear();
    LCDStr ( 0, "Real time play", 0 );
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();
      
    Vocoder_Init();
    Vocoder_Start();
    Start_Playing = REAL_VOICE;
  
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET)
    {    
      if(Start_Encoding == 1)
      {
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* Copy the encoded data into the bit-stream struct */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);

        Start_Encoding = 0;	
      }
      else if(Start_Encoding == 2)
      {
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* Copy the encoded data into the bit-stream struct */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);
        
        Start_Encoding = 0;
      }
    }
    
    Start_Encoding = 0;
    inBuffer = IN_Buffer[0];
    outBuffer = OUT_Buffer[0];
    Vocoder_Stop();
    goto MENU_LABEL;
  }

  /* Record voice   ------------------------------------------------------------*/
  if(MenuPos == 3) 
  {
    LCDClear();
    LCDStr ( 0, "Record voice  ", 0 );
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "              ", 0 );
    LCDUpdate();
    
    Vocoder_Init();
    Voice_Recording_Init();

    Recording = 1;
    Encoded_Frames=0;

    LCDClear();
    LCDStr ( 0, "Recording...  ", 0 );
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();

    Voice_Recording_Start();
    
    while(Recording && (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    {	 
      if(Flash_Address == RECORDING_END_ADDRESS)
      {
        Recording = 0;
        Start_Encoding = 0;	 
      }
      
      if(Start_Encoding == 1)
      {
        int i;
        
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        
        for(i=0;i<ENCODED_FRAME_SIZE;i+=2)
        {
          FLASH_ProgramHalfWord(Flash_Address, (out_bytes[i]|out_bytes[i+1]<<8));
          Flash_Address+=2;
        }
        
        Start_Encoding = 0;
      }
      else if(Start_Encoding == 2)
      {
        int i;
        
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        
        for(i=0;i<ENCODED_FRAME_SIZE;i+=2)
        {
          FLASH_ProgramHalfWord(Flash_Address, (out_bytes[i+1]<<8)|out_bytes[i]);
          Flash_Address+=2;
	}
        
        Start_Encoding = 0;
      } 
    }
   
    inBuffer = IN_Buffer[0];
    outBuffer = OUT_Buffer[0];
    Recording = 0;
    Start_Encoding = 0;
    Flash_Address = RECORDING_START_ADDRESS;  
    pFlash = (uint8_t*)RECORDING_START_ADDRESS;
    Voice_Recording_Stop();
    goto MENU_LABEL;
  }
  
  /* Play recorded voice   -----------------------------------------------------*/
  if(MenuPos == 4) 
  {
    Playing = 1;  
    Vocoder_Init();
    LCDClear();
    LCDStr ( 0, "Playing...    ", 0 );
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();
  
    Voice_Playing_Start();
    Start_Decoding = 1;
  
    while ((NB_Frames!=Encoded_Frames)&& (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    {
      if(Start_Decoding == 1)
      {
        int i;
	
        /* Read 20 data from the flash memory */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* Copy the encoded data into the bit-stream struct */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[0]);
        /* Signal the end of the decoding */
        Start_Decoding = 0;
        
        NB_Frames++;
      }
      else if(Start_Decoding == 2)
      {
        int i;	
        
        /* Read 20 data from the flash memory */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* Copy the encoded data into the bit-stream struct */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* Decode the data */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[1]);
        /* Signal the end of the decoding */
        Start_Decoding = 0;
        
        NB_Frames++;
      }
    }
    
    Start_Encoding = 0;
    Start_Decoding = 0;
    Playing =0;
    NB_Frames = 0;
    inBuffer = IN_Buffer[0];
    outBuffer = OUT_Buffer[0];
    pFlash = (uint8_t*)RECORDING_START_ADDRESS;
    Flash_Address = RECORDING_START_ADDRESS;
    goto MENU_LABEL;
  }
  
  while(1);
}



/**
  * @brief  Initializes the demonstration application.
  * @param  None
  * @retval : None
  */
void Demo_Init(void)
{
  ErrorStatus HSEStartUpStatus;
  
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
  }
 
  /* TIM2 and TIM3 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 , ENABLE);

   /* Enable GPIOA, GPIOC, ADC1 , AFIO and TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

  /* Interrupt Configuration */
  InterruptConfig();

}



/**
  * @brief  Configures the used IRQ Channels and sets their priority.
  * @param  None
  * @retval : None
  */
void InterruptConfig(void)
{ 

   /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_SetPriorityGrouping(6); 

	 /* Enable the TIM2 Interrupt */
  NVIC_SetPriority(TIM2_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM2_IRQn);
  /* Enable the TIM3 Interrupt */
 	 /* Enable the TIM2 Interrupt */
  NVIC_SetPriority(TIM3_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM3_IRQn);
}



/**
  * @brief  Initializes the speex codec
  * @param  None
  * @retval : None
  */
void Speex_Init(void)
{
  /* Speex encoding initializations */ 
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
  /* speex decoding intilalization */
  dec_state = speex_decoder_init(&speex_nb_mode);
  speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);
}

/*************************************************************************
* Function Name  : InitJoystick
* Description    : Init joystick position
* Input          : None
* Output         : None
* Return         : None
*************************************************************************/
void InitJoystick(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef	  ADC_InitStructure;
  /* Set variables asociates */
  JoyPos = 0;
  MenuPos = 1;

  /* ADC Init */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOA, ENABLE);
  ADC_DeInit(ADC1);

  /* RA1 - analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  /* ADC Structure Initialization */
  ADC_StructInit(&ADC_InitStructure);

  /* Preinit */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Enable the ADC */
  ADC_Cmd(ADC1, ENABLE);

  /* BUTTON CENTER as input */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* B1 as input */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*************************************************************************
* Function Name  : UpdateMenu
* Description    : Black mark of menu position
* Input          : - pos: Position of menue.
* Output         : None
* Return         : Joystick position
**************************************************************************/
void UpdateMenu(uint8_t pos) 
{
  uint8_t MARK1[4] = {1,0,0,0};
  uint8_t MARK2[4] = {0,1,0,0};
  uint8_t MARK3[4] = {0,0,1,0};
  uint8_t MARK4[4] = {0,0,0,1};

  uint8_t *mark;
  
  if(pos==1)
  {
    mark = MARK1;
  }
  else if(pos==2)
  {
    mark = MARK2;
  }
  else if(pos==3)
  {
    mark = MARK3;
  }
  else// if(pos==4)
  {
    mark = MARK4;
  }
    
  LCDClear();
  LCDStr ( 0, Menu[0], 0);
  LCDStr ( 2, Menu[1], mark[0]);
  LCDStr ( 3, Menu[2], mark[1]);
  LCDStr ( 4, Menu[3], mark[2]);
  LCDStr ( 5, Menu[4], mark[3]);
  LCDUpdate();
}

/*************************************************************************
* Function Name  : GetJoystickPosition
* Description    : Measure ADC and return joystick position
* Input          : None
* Output         : None
* Return         : Joistick position
 *************************************************************************/
uint8_t GetJoystickPosition (void) 
{
  /* Configure chanel */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);

  /* Start the conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Wait until conversion completion */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  /* Get the conversion value */
  ADCValue=  ADC_GetConversionValue(ADC1);

  if( (ADCValue>(UP_VALUE-DIVERSION))&&(ADCValue<(UP_VALUE+DIVERSION)) )       { return KEY_UP; }
  if( (ADCValue>(DOWN_VALUE-DIVERSION))&&(ADCValue<(DOWN_VALUE+DIVERSION)) )   { return KEY_DOWN; }
  if( (ADCValue>(LEFT_VALUE-DIVERSION))&&(ADCValue<(LEFT_VALUE+DIVERSION)) )   { return KEY_LEFT; }
  if( (ADCValue>(RIGHT_VALUE-DIVERSION))&&(ADCValue<(RIGHT_VALUE+DIVERSION)) ) { return KEY_RIGHT; }

  if((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) == Bit_SET) return KEY_CENTER;

  return KEY_NONE;
}



/**
  * @brief  Ovveride the _speex_putc function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_putc(int ch, void *file)
{
  while(1)
  {
  };
}



/**
  * @brief  Ovveride the _speex_fatal function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_fatal(const char *str, const char *file, int line)
{
  while(1)
  {
  };
}

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
