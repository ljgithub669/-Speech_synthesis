/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/main.c 
  * @笔者MCD应用团队
  * @版本  V2.0.0
  * @date     04/27/2009
  * @简短的主程序体
  ******************************************************************************
  * 修改：.S启动文件的 Heap_Size       EQU     0x00002000;修改为4KB，不然会硬件异常
  *	PA.8为 TIM1 PWM语音输出，PA.1为语音输入。在Vocoder.C配置
  *	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET要去除，不然会放不出声音
  * 
  *	只在 STM32F103C8T6 上调出了语音
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* 包括 ------------------------------------------------------------------*/
#include "main.h"
#include "voice.h"
//#include "lcd.h"去除LCD
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <speex/speex.h>
#include"arch.h"
////////////////////////
#include "delay.h"
#include "sys.h"
#include "oled.h"////加入0.96寸OLED

/** @addtogroup Speex语音声码器STM32F103 STK
  * @{
  */ 


/* 专用的typedef -----------------------------------------------------------*/
/* 专用定义 ------------------------------------------------------------*/
#define ALL_FRAMES      300   /* 编码的男声长度 */

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

/* 专用宏 -------------------------------------------------------------*/
/* 专用变量 ---------------------------------------------------------*/
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

SpeexBits bits;/* 保存位，这样他们可以读取和写入的Speex的例程 */
void *enc_state;
void *dec_state;/* 保存编码器和解码器的状态 */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX参数，必须保持不变 */

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

/* 专用函数原型 -----------------------------------------------*/
void Demo_Init(void);
void Speex_Init(void);//Speex初始化
void InterruptConfig(void);//中断配置
void InitJoystick(void);//初始化操纵杆(五向键)
//void UpdateMenu(unsigned char pos);//更新菜单
uint8_t GetJoystickPosition (void);//获取操纵杆位置

/* 私有函数 ---------------------------------------------------------*/



/**
  * @简短的主程序。
  * @参数  无
  * @retval : None
  */
int main(void)
{
	delay_init();	    	 //延时函数初始化	
  Demo_Init();
  Speex_Init();
    
  /* OLED 初始化 */
  OLED_Init();	//加入0.96寸OLED
  
  //LCDContrast(0x45);
  
  /* 显示欢迎信息 */
  OLED_ShowString(0,0,"by Speex codec"); 
  //OLED_ShowNum(30,6,adcx,4,16);//OLED_ShowNum(x列,y行,32位变量,显示数字个数,字体16/12)
  /*
  LCDClear();
  LCDStr ( 0, "**************", 0 );
  LCDStr ( 1, "  Welcome to  ", 0 ); //"欢迎来到"
  LCDStr ( 2, "   STM32F10x  ", 0 );
  LCDStr ( 3, " vocoder demo ", 0 ); //"声码器演示"
  LCDStr ( 4, "by Speex codec", 0 ); //"通过Speex编解码器"
  LCDStr ( 5, "**************", 0 );
  LCDUpdate();

   延迟 */
 for(dly=0; dly<LCD_DELAY; dly++);	//Speex初始化完成后第一步会跳到这里
 
	MenuPos = 1;					//设置第一个演示：播放一段男声

MENU_LABEL:

  /* 初始化操纵杆*/
  InitJoystick();
  
  /* 绘制菜单 */
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
  /* 播放欢迎信息 ---------------------------------------------------*/
  if(MenuPos == 1) 
  {
    int i;
	OLED_ShowString(0,0,"1 male voice");
   /* LCDClear();
    LCDStr ( 0, "Stored encoded", 0 ); //"存储的编码"
    LCDStr ( 1, "  male voice  ", 0 ); //"男声"
    LCDStr ( 2, "**************", 0 );
    LCDStr ( 4, "              ", 0 );
    LCDStr ( 5, "exit          ", 0 ); //"出口"
    LCDUpdate();												*/

    Start_Playing = MALE_VOICE;
    Vocoder_Init();
  
    /* 正准备两个缓冲区 of 解码后的数据：*/
    /* 第一个，*/
    for(i=0;i<ENCODED_FRAME_SIZE; i++)//ENCODED_FRAME_SIZE为 20，即压缩后1帧的大小
    {
      input_bytes[i] = male_voice[sample_index++];// male_voice ：voice.h里面压缩后的语音数组
    } 
      
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
  
    /* 和第二个。 */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = male_voice[sample_index++];
    } 
 
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);

    NB_Frames++;

    Vocoder_Start();
  
    /*现在我们等到缓冲器的播放重新编码...*/
    // while((NB_Frames < ALL_FRAMES)&&(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET))
    while(NB_Frames < ALL_FRAMES) //ALL_FRAMES ：300个编码的男声长度
	{
      if(Start_Decoding == 1) /* 我们开始第一个缓冲区解码 */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* 的编码数据复制到比特流结构 */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* 解码该数据 */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
      if(Start_Decoding == 2) /* 我们开始第二个缓冲解码 */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = male_voice[sample_index++];
        }
        
        /* 的编码数据复制到比特流结构 */
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        /* 解码该数据 */
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

/* 		实时播放 
  -----话筒信号由PA.1输入；PA.8输出 PWM 语音-----*/
  if(MenuPos == 2) 
  { 
	OLED_ShowString(0,2,"2 Real time play");//“实时播放”
	
    Vocoder_Init();
    Vocoder_Start();
    Start_Playing = REAL_VOICE;
  
    //while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET)
    while(MenuPos == 2)
			
		{    
      if(Start_Encoding == 1)//开始_编码=1
      {
        /* 刷新在结构中的所有位，以便我们可以编码一个新的帧 */
        speex_bits_reset(&bits);
        /* 编码帧 */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);//IN_Buffer：输入的数据帧，
        /* 复制位字符数组可以被解码 */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* 的编码数据复制到比特流结构 */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* 解码该数据 */
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);//OUT_Buffer：输出的数据帧

        Start_Encoding = 0;	
      }
      else if(Start_Encoding == 2)
      {
        /* 刷新在结构中的所有位，所以我们可以编码一个新的帧 */
        speex_bits_reset(&bits);
        /* 编码帧 */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* 复制位字符数组可以被解码 */
        speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* 的编码数据复制到比特流结构 */
        speex_bits_read_from(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
        /* 解码该数据 */
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

  /* 录制的声音   ------------------------------------------------------------*/
  if(MenuPos == 3) 
  { 
	OLED_ShowString(0,4,"3 Record voice");	//“记录声音”

    Vocoder_Init();
    Voice_Recording_Init();

    Recording = 1;
    Encoded_Frames=0;
	OLED_ShowString(0,4,"3 Recording... ");	//“记录... ”
	/*
    LCDClear();
    LCDStr ( 0, "Recording...  ", 0 ); //“记录... ”
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
        
        /* 刷新在结构中的所有位，所以我们可以编码一个新的帧 */
        speex_bits_reset(&bits);
        /* 编码帧*/
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[0], &bits);
        /* 复制位字符数组可以被解码 */
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
        
        /* 刷新在结构中的所有位，所以我们可以编码一个新的帧 */
        speex_bits_reset(&bits);
        /* 编码帧 */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[1], &bits);
        /* 复制位字符数组可以被解码 */
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
  
  /* 播放录制的语音   -----------------------------------------------------*/
  if(MenuPos == 4) 
  { 
	OLED_ShowString(0,6,"4 Playing... ");	// "播放..."
/*
    Playing = 1;  
    Vocoder_Init();
    LCDClear();
    LCDStr ( 0, "Playing...    ", 0 ); // "播放..."
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
	
        /* 从闪存读取20个数据 */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* 的编码数据复制到比特流结构 */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* 解码该数据 */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[0]);
        /* 信号解码的结束 */
        Start_Decoding = 0;
        
        NB_Frames++;
      }
      else if(Start_Decoding == 2)
      {
        int i;	
        
        /* 从闪存读取20个数据 */
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = *pFlash++;
        } 
  
        /* 的编码数据复制到比特流结构 */    
        speex_bits_read_from(&bits, (char*)input_bytes, ENCODED_FRAME_SIZE);  
        /* 解码该数据 */
        speex_decode_int(dec_state, &bits, (spx_int16_t *)OUT_Buffer[1]);
        /* 信号解码的结束 */
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
  * @简短的初始化示范应用。
  * @参数  无
  * @retval : None
  */
void Demo_Init(void)
{
  
 
  /* TIM2和TIM3时钟启用 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 , ENABLE);

   /* 启用GPIOA，GPIOC，ADC1，AFIO和TIM1时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

  /* 中断配置 */
  InterruptConfig();

}



/**
  * @简短的配置所使用的IRQ通道，并设置它们的优先级。
  * @param  None
  * @retval : None
  */
void InterruptConfig(void)
{ 

   /* 1位占先的优先级，3位为子优先 */
  NVIC_SetPriorityGrouping(6); 

	 /* 启用TIM2中断 */
  NVIC_SetPriority(TIM2_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM2_IRQn);
  /* 启用TIM3中断 */
 	 /* 启用TIM2中断 */
  NVIC_SetPriority(TIM3_IRQn, 0x00); /* 0x00 = 0x01 << 3 | (0x00 & 0x7*/
  NVIC_EnableIRQ(TIM3_IRQn);
}



/**
  * @简短的初始化Speex编解码器
  * @param  None
  * @retval : None
  */
void Speex_Init(void)
{
  /* Speex语音编码初始化 */
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
  /* Speex语音解码初始化 */
  dec_state = speex_decoder_init(&speex_nb_mode);	//用speex_wb_mode代替speex_nb_mode，可转换为宽带（windband）解码。
  speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);//&enh：知觉增强
}

/*************************************************************************
* 功能名称 : 初始化操纵杆
* 描述    : 初始化操纵杆位置
* Input          : None
* Output         : None
* Return         : None
*************************************************************************/
void InitJoystick(void) 
{
	/*
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef	  ADC_InitStructure;
  /* 设置变量asociates *
  JoyPos = 0;
  MenuPos = 1;

  /* ADC 初始化 *
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOA, ENABLE);
  ADC_DeInit(ADC1);

  /* RA1 - 模拟输入 *
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  /* ADC 结构初始化 *
  ADC_StructInit(&ADC_InitStructure);

  /* Preinit *
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* 启用 ADC *
  ADC_Cmd(ADC1, ENABLE);

  /* BUTTON 中心输入 *
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
* 功能名称  : 更新菜单
* 描述   : 菜单位置黑标
* Input          : - POS：MENUE的位置。
* Output         : None
* 返回         : 操纵杆位置
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
* 功能名称  : 获取操纵杆位置
* 描述   : 测量ADC和返回操纵杆位置
* Input          : None
* Output         : None
* 返回           : 操纵杆位置
 *************************************************************************/
uint8_t GetJoystickPosition (void) 
{
  /* 配置 chanel *
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);

  /* 开始对话 *
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* 等到转换完成 *
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  /* 获取转换值 *
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
  * @简要覆盖的Speex语音库的_speex_putc功能
  * @参数 无
  * @retval : None
  */
void _speex_putc(int ch, void *file)
{
  while(1)
  {
  };
}



/**
  * @简要覆盖的Speex语音库的_speex_fatal功能
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
  * @简要报告源文件名和源代码行数
  *   在那里发生了assert_param错误。
  * @参数文件：指向源文件名
  * @参数行：assert_param错误行源数量
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* 用户可以添加自己的执行情况报告的文件名和行号，
     例如：printf的（“错误的参数值：文件％线％D\ r\?的”，文件，行） *

  /* 无限循环*/
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 


/******************* (C) 文件的版权2009年意法半导体***** END****/
