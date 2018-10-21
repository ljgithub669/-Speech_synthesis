/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/main.c 
  * @����MCDӦ���Ŷ�
  * @�汾  V2.0.0
  * @date     04/27/2009
  * @��̵���������
  ******************************************************************************
  * �޸ģ�.S�����ļ��� Heap_Size       EQU     0x00002000;�޸�Ϊ4KB����Ȼ��Ӳ���쳣
  *	PA.8Ϊ TIM1 PWM���������PA.1Ϊ�������롣��Vocoder.C����
  *	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESETҪȥ������Ȼ��Ų�������
  * 
  *	ֻ�� STM32F103C8T6 �ϵ���������
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* ���� ------------------------------------------------------------------*/
#include "main.h"
#include "voice.h"
//#include "lcd.h"ȥ��LCD
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <speex/speex.h>
#include"arch.h"
////////////////////////
#include "delay.h"
#include "sys.h"
#include "oled.h"////����0.96��OLED

/** @addtogroup Speex����������STM32F103 STK
  * @{
  */ 


/* ר�õ�typedef -----------------------------------------------------------*/
/* ר�ö��� ------------------------------------------------------------*/
#define ALL_FRAMES      300   /* ������������� */

#define LCD_DELAY       0x7FFFFF//8388607
#define JOYS_DELAY      0x10FFFF//1114111

#define KEY_NONE        0
#define KEY_UP          1
#define KEY_DOWN        2
#define KEY_LEFT        3
#define KEY_RIGHT       4
#define KEY_CENTER      5
/*
#define UP_VALUE        960
#define DOWN_VALUE      190
#define LEFT_VALUE      1990
#define RIGHT_VALUE     470
#define DIVERSION       30
*/
#define MENU_NUMB      5
#define MENU_WIDTH      15

/* ר�ú� -------------------------------------------------------------*/
/* ר�ñ��� ---------------------------------------------------------*/
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

SpeexBits bits;/* ����λ���������ǿ��Զ�ȡ��д���Speex������ */
void *enc_state;
void *dec_state;/* ����������ͽ�������״̬ */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX���������뱣�ֲ��� */

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

/* ר�ú���ԭ�� -----------------------------------------------*/
void Demo_Init(void);
void Speex_Init(void);//Speex��ʼ��
void InterruptConfig(void);//�ж�����
void InitJoystick(void);//��ʼ�����ݸ�(�����)
//void UpdateMenu(unsigned char pos);//���²˵�
uint8_t GetJoystickPosition (void);//��ȡ���ݸ�λ��

/* ˽�к��� ---------------------------------------------------------*/



/**
  * @��̵�������
  * @����  ��
  * @retval : None
  */
int main(void)
{
	delay_init();	    	 //��ʱ������ʼ��	
  Demo_Init();
  Speex_Init();
    
  /* OLED ��ʼ�� */
  OLED_Init();	//����0.96��OLED
  
  //LCDContrast(0x45);
  
  /* ��ʾ��ӭ��Ϣ */
  OLED_ShowString(0,0,"by Speex codec"); 
  //OLED_ShowNum(30,6,adcx,4,16);//OLED_ShowNum(x��,y��,32λ����,��ʾ���ָ���,����16/12)
  /*
  LCDClear();
  LCDStr ( 0, "**************", 0 );
  LCDStr ( 1, "  Welcome to  ", 0 ); //"��ӭ����"
  LCDStr ( 2, "   STM32F10x  ", 0 );
  LCDStr ( 3, " vocoder demo ", 0 ); //"��������ʾ"
  LCDStr ( 4, "by Speex codec", 0 ); //"ͨ��Speex�������"
  LCDStr ( 5, "**************", 0 );
  LCDUpdate();

   �ӳ� */
 for(dly=0; dly<LCD_DELAY; dly++);	//Speex��ʼ����ɺ��һ������������
 
	MenuPos = 1;					//���õ�һ����ʾ������һ������

MENU_LABEL:

  /* ��ʼ�����ݸ�*/
  InitJoystick();
  
  /* ���Ʋ˵� */
  //UpdateMenu(MenuPos);

 // while(1) 
  //{
		
	/*
    JoyPos = GetJoystickPosition();
		
    if(JoyPos==KEY_UP)
    {
      MenuPos--;
      if((MenuPos<1)) MenuPos = 1;
      //UpdateMenu(MenuPos);
      for(dly=0; dly<JOYS_DELAY; dly++);
    }

    if(JoyPos==KEY_DOWN)
    {
      MenuPos++;
      if((MenuPos>4)) MenuPos = 4;
      //UpdateMenu(MenuPos);
      for(dly=0; dly<JOYS_DELAY; dly++);
    }

    if(JoyPos==KEY_CENTER) break;
		
		
  }
		*/
  /* ���Ż�ӭ��Ϣ ---------------------------------------------------*/
  if(MenuPos == 1) 
  {
    int i;
	OLED_ShowString(0,0,"1 male voice");
   /* LCDClear();
    LCDStr ( 0, "Stored encoded", 0 ); //"�洢�ı���"
    LCDStr ( 1, "  male voice  ", 0 ); //"����"
    LCDStr ( 2, "**************", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 ); //"����"
    LCDUpdate();												*/

    Start_Playing = MALE_VOICE;
    Vocoder_Init();
  
    /* ��׼������������ of ���������ݣ�*/
    /* ��һ����*/
    for(i=0;i<ENCODED_FRAME_SIZE; i++)//ENCODED_FRAME_SIZEΪ 20����ѹ����1֡�Ĵ�С
    {
      input_bytes[i] = male_voice[sample_index++];// male_voice ��voice.h����ѹ�������������
    } 
      
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
  
    /* �͵ڶ����� */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = male_voice[sample_index++];
    } 
 
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);

    NB_Frames++;

    Vocoder_Start();
  
    /*�������ǵȵ��������Ĳ������±���...*/
    // while((NB_Frames < ALL_FRAMES)&&(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    while(NB_Frames < ALL_FRAMES) //ALL_FRAMES ��300���������������
	{
      if(Start_Decoding == 1) /* ���ǿ�ʼ��һ������������ */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* �ı������ݸ��Ƶ��������ṹ */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* ��������� */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
      if(Start_Decoding == 2) /* ���ǿ�ʼ�ڶ���������� */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* �ı������ݸ��Ƶ��������ṹ */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* ��������� */
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
//}

/* 		ʵʱ���� 
  -----��Ͳ�ź���PA.1���룻PA.8��� PWM ����-----*/
  if(MenuPos == 2) 
  { 
	OLED_ShowString(0,2,"2 Real time play");//��ʵʱ���š�
	
    Vocoder_Init();
    Vocoder_Start();
    Start_Playing = REAL_VOICE;
  
    //while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET)
    while(MenuPos == 2)
			
		{    
      if(Start_Encoding == 1)//��ʼ_����=1
      {
        /* ˢ���ڽṹ�е�����λ���Ա����ǿ��Ա���һ���µ�֡ */
        speex_bits_reset(&bits);
        /* ����֡ */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);//IN_Buffer�����������֡��
        /* ����λ�ַ�������Ա����� */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* �ı������ݸ��Ƶ��������ṹ */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* ��������� */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);//OUT_Buffer�����������֡

        Start_Encoding = 0;	
      }
      else if(Start_Encoding == 2)
      {
        /* ˢ���ڽṹ�е�����λ���������ǿ��Ա���һ���µ�֡ */
        speex_bits_reset(&bits);
        /* ����֡ */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* ����λ�ַ�������Ա����� */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* �ı������ݸ��Ƶ��������ṹ */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* ��������� */
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

  /* ¼�Ƶ�����   ------------------------------------------------------------*/
  if(MenuPos == 3) 
  { 
	OLED_ShowString(0,4,"3 Record voice");	//����¼������

    Vocoder_Init();
    Voice_Recording_Init();

    Recording = 1;
    Encoded_Frames=0;
	OLED_ShowString(0,4,"3 Recording... ");	//����¼... ��
	/*
    LCDClear();
    LCDStr ( 0, "Recording...  ", 0 ); //����¼... ��
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();
*/
    Voice_Recording_Start();
    
    //while(Recording && (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    while(Recording)
	{	 
      if(Flash_Address == RECORDING_END_ADDRESS)
      {
        Recording = 0;
        Start_Encoding = 0;	 
      }
      
      if(Start_Encoding == 1)
      {
        int i;
        
        /* ˢ���ڽṹ�е�����λ���������ǿ��Ա���һ���µ�֡ */
        speex_bits_reset(&bits);
        /* ����֡*/
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);
        /* ����λ�ַ�������Ա����� */
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
        
        /* ˢ���ڽṹ�е�����λ���������ǿ��Ա���һ���µ�֡ */
        speex_bits_reset(&bits);
        /* ����֡ */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* ����λ�ַ�������Ա����� */
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
  
  /* ����¼�Ƶ�����   -----------------------------------------------------*/
  if(MenuPos == 4) 
  { 
	OLED_ShowString(0,6,"4 Playing... ");	// "����..."
/*
    Playing = 1;  
    Vocoder_Init();
    LCDClear();
    LCDStr ( 0, "Playing...    ", 0 ); // "����..."
    LCDStr ( 1, "**************", 0 );
    LCDStr ( 2, "              ", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 );
    LCDUpdate();
  */
    Voice_Playing_Start();
    Start_Decoding = 1;
  
    //while ((NB_Frames!=Encoded_Frames)&& (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
   while (NB_Frames!=Encoded_Frames) 
	{
      if(Start_Decoding == 1)
      {
        int i;
	
        /* �������ȡ20������ */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* �ı������ݸ��Ƶ��������ṹ */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* ��������� */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[0]);
        /* �źŽ���Ľ��� */
        Start_Decoding = 0;
        
        NB_Frames++;
      }
      else if(Start_Decoding == 2)
      {
        int i;	
        
        /* �������ȡ20������ */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* �ı������ݸ��Ƶ��������ṹ */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* ��������� */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[1]);
        /* �źŽ���Ľ��� */
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
  * @��̵ĳ�ʼ��ʾ��Ӧ�á�
  * @����  ��
  * @retval : None
  */
void Demo_Init(void)
{
  
 
  /* TIM2��TIM3ʱ������ */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 , ENABLE);

   /* ����GPIOA��GPIOC��ADC1��AFIO��TIM1ʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

  /* �ж����� */
  InterruptConfig();

}



/**
  * @��̵�������ʹ�õ�IRQͨ�������������ǵ����ȼ���
  * @param  None
  * @retval : None
  */
void InterruptConfig(void)
{ 

   /* 1λռ�ȵ����ȼ���3λΪ������ */
  NVIC_SetPriorityGrouping(6); 

	 /* ����TIM2�ж� */
  NVIC_SetPriority(TIM2_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM2_IRQn);
  /* ����TIM3�ж� */
 	 /* ����TIM2�ж� */
  NVIC_SetPriority(TIM3_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM3_IRQn);
}



/**
  * @��̵ĳ�ʼ��Speex�������
  * @param  None
  * @retval : None
  */
void Speex_Init(void)
{
  /* Speex���������ʼ�� */
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
  /* Speex���������ʼ�� */
  dec_state = speex_decoder_init(&speex_nb_mode);	//��speex_wb_mode����speex_nb_mode����ת��Ϊ�����windband�����롣
  speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);//&enh��֪����ǿ
}

/*************************************************************************
* �������� : ��ʼ�����ݸ�
* ����    : ��ʼ�����ݸ�λ��
* Input          : None
* Output         : None
* Return         : None
*************************************************************************/
void InitJoystick(void) 
{
	/*
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef	  ADC_InitStructure;
  /* ���ñ���asociates *
  JoyPos = 0;
  MenuPos = 1;

  /* ADC ��ʼ�� *
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOA, ENABLE);
  ADC_DeInit(ADC1);

  /* RA1 - ģ������ *
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  /* ADC �ṹ��ʼ�� *
  ADC_StructInit(&ADC_InitStructure);

  /* Preinit *
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ���� ADC *
  ADC_Cmd(ADC1, ENABLE);

  /* BUTTON �������� *
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* B1 as input *
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
		*/
}

/*************************************************************************
* ��������  : ���²˵�
* ����   : �˵�λ�úڱ�
* Input          : - POS��MENUE��λ�á�
* Output         : None
* ����         : ���ݸ�λ��
**************************************************************************
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
    
 /* LCDClear();
  LCDStr ( 0, Menu[0], 0);
  LCDStr ( 2, Menu[1], mark[0]);
  LCDStr ( 3, Menu[2], mark[1]);
  LCDStr ( 4, Menu[3], mark[2]);
  LCDStr ( 5, Menu[4], mark[3]);
  LCDUpdate();	
} *
}

/*************************************************************************
* ��������  : ��ȡ���ݸ�λ��
* ����   : ����ADC�ͷ��ز��ݸ�λ��
* Input          : None
* Output         : None
* ����           : ���ݸ�λ��
 *************************************************************************/
uint8_t GetJoystickPosition (void) 
{
  /* ���� chanel *
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);

  /* ��ʼ�Ի� *
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* �ȵ�ת����� *
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  /* ��ȡת��ֵ *
  ADCValue=  ADC_GetConversionValue(ADC1);

  if( (ADCValue>(UP_VALUE-DIVERSION))&&(ADCValue<(UP_VALUE+DIVERSION)) )       { return KEY_UP; }
  if( (ADCValue>(DOWN_VALUE-DIVERSION))&&(ADCValue<(DOWN_VALUE+DIVERSION)) )   { return KEY_DOWN; }
  if( (ADCValue>(LEFT_VALUE-DIVERSION))&&(ADCValue<(LEFT_VALUE+DIVERSION)) )   { return KEY_LEFT; }
  if( (ADCValue>(RIGHT_VALUE-DIVERSION))&&(ADCValue<(RIGHT_VALUE+DIVERSION)) ) { return KEY_RIGHT; }

  if((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) == Bit_SET) return KEY_CENTER;
	*/
  return KEY_NONE;
}



/**
  * @��Ҫ���ǵ�Speex�������_speex_putc����
  * @���� ��
  * @retval : None
  */
void _speex_putc(int ch, void *file)
{
  while(1)
  {
  };
}



/**
  * @��Ҫ���ǵ�Speex�������_speex_fatal����
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
  * @��Ҫ����Դ�ļ�����Դ��������
  *   �����﷢����assert_param����
  * @�����ļ���ָ��Դ�ļ���
  * @�����У�assert_param������Դ����
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* �û���������Լ���ִ�����������ļ������кţ�
     ���磺printf�ģ�������Ĳ���ֵ���ļ����ߣ�D\ r\?�ġ����ļ����У� *

  /* ����ѭ��*/
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 


/******************* (C) �ļ��İ�Ȩ2009���ⷨ�뵼��***** END****/
