/**
  ******************************************************************************
  * @file SpeexVocoder_STM32F103_STK/src/lcd.c
  * @author   OLIMEX
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    LCD module
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
  * <h2><center>&copy; Copyright (C) 2005 OLIMEX  LTD.</center></h2>
  */

#include "lcd.h"
#include "stm32f10x.h"
//#include "arm_comm.h"

// LCD memory index
unsigned int  LcdMemIdx;

// represent LCD matrix
unsigned char  LcdMemory[LCD_CACHE_SIZE];

// simple delay
void Delay(__IO unsigned long a) { while (--a!=0); }


/****************************************************************************/
/*  Init LCD Controler                                                      */
/*  Function : LCDInit                                                      */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDInit(void)
{

  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable SPI1 and GPIOA clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  // Configure SPI1 pins: NSS, SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure PB2 as Output push-pull, used as D/C
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // D/C high
  GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);

  // Configure PC7 and PC10 as Output push-pull, used as LCD_RES and LCD_E
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  // LCD_E - disable
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);
  // Set Reset pin (active low)
  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET);

  // SPI1 configuration
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  // Enable SPI1
  SPI_Cmd(SPI1, ENABLE);

  // Toggle display reset pin.
  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET);
  Delay(10000);
  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET);
  Delay(10000);

  // Send sequence of command
  LCDSend( 0x21, SEND_CMD );  // LCD Extended Commands.
  LCDSend( 0xC8, SEND_CMD );  // Set LCD Vop (Contrast).
  LCDSend( 0x06, SEND_CMD );  // Set Temp coefficent.
  LCDSend( 0x13, SEND_CMD );  // LCD bias mode 1:48.
  LCDSend( 0x20, SEND_CMD );  // LCD Standard Commands, Horizontal addressing mode.
  LCDSend( 0x08, SEND_CMD );  // LCD blank
  LCDSend( 0x0C, SEND_CMD );  // LCD in normal mode.

  // Clear and Update
  LCDClear();
  LCDUpdate();

}

/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDSend                                                      */
/*      Parameters                                                          */
/*          Input   :  data and  SEND_CHR or SEND_CMD                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDSend(unsigned char data, unsigned char cd) {

  // Enable display controller (active low) -> LCD_E low
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);

  // command or data - D/S low or high
  if(cd == SEND_CHR) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
  }
  else {
    GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
  }

  ///// SEND SPI /////
  // Loop while DR register in not emplty
  /*while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);*/
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  // Send byte through the SPI1 peripheral
  /*SPI_SendData(SPI1, data);	*/
  SPI_I2S_SendData(SPI1, data);

  // Wait to receive a byte
  /*while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);*/
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
			 
  // Return the byte read from the SPI bus
  // return SPI_ReceiveData(SPI1);
  ///// SEND SPI END /////


  // Disable display controller -> LCD_E high
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);
}

/****************************************************************************/
/*  Update LCD memory                                                       */
/*  Function : LCDUpdate                                                    */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDUpdate ( void )
{

  int i;

  //  Set base address X=0 Y=0
  LCDSend(0x80, SEND_CMD );
  LCDSend(0x40, SEND_CMD );

  //  Serialize the video buffer.
  for (i=0; i<LCD_CACHE_SIZE; i++) {
    LCDSend( LcdMemory[i], SEND_CHR );
  }
}

/****************************************************************************/
/*  Clear LCD                                                               */
/*  Function : LCDClear                                                     */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDClear(void) {

  int i;

  // loop all cashe array
  for (i=0; i<LCD_CACHE_SIZE; i++)
  {
     LcdMemory[i] = 0x0;
  }

}




/****************************************************************************/
/*  Change LCD Pixel mode                                                   */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDPixel (unsigned char x, unsigned char y, unsigned char mode )
{
    unsigned int    index   = 0;
    unsigned char   offset  = 0;
    unsigned char   data    = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = ((y / 8) * 84) + x;
    offset  = y - ((y / 8) * 8);

    data = LcdMemory[index];

    if ( mode == PIXEL_OFF )
    {
        data &= (~(0x01 << offset));
    }
    else if ( mode == PIXEL_ON )
    {
        data |= (0x01 << offset);
    }
    else if ( mode  == PIXEL_XOR )
    {
        data ^= (0x01 << offset);
    }

    LcdMemory[index] = data;

}

/****************************************************************************/
/*  Write char at x position on y row                                       */
/*  Function : LCDChrXY                                                     */
/*      Parameters                                                          */
/*          Input   :  pos, row, char                                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChrXY (unsigned char x, unsigned char y, unsigned char ch )
{
    unsigned int    index   = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = (x*48 + y*48*14)/8 ;

    for ( i = 0; i < 5; i++ )
    {
      LcdMemory[index] = FontLookup[ch - 32][i] << 1;
      index++;
    }

}


/****************************************************************************/
/*  Write char at x position on y row - inverse                             */
/*  Function : LCDChrXY                                                     */
/*      Parameters                                                          */
/*          Input   :  pos, row, char                                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChrXYInverse (unsigned char x, unsigned char y, unsigned char ch )
{
    unsigned int    index   = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = (x*48 + y*48*14)/8 ;

    for ( i = 0; i < 5; i++ )
    {
      LcdMemory[index] = ~(FontLookup[ch - 32][i]);
      index++;

      if(i==4)
        LcdMemory[index] = 0xFF;
    }

}


/****************************************************************************/
/*  Set LCD Contrast                                                        */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDContrast(unsigned char contrast) {

    //  LCD Extended Commands.
    LCDSend( 0x21, SEND_CMD );

    // Set LCD Vop (Contrast).
    LCDSend( 0x80 | contrast, SEND_CMD );

    //  LCD Standard Commands, horizontal addressing mode.
    LCDSend( 0x20, SEND_CMD );
}


/****************************************************************************/
/*  Send string to LCD                                                      */
/*  Function : LCDStr                                                       */
/*      Parameters                                                          */
/*          Input   :  row, text, inversion                                 */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDStr(unsigned char row, unsigned char *dataPtr, unsigned char inv ) {

  // variable for X coordinate
  unsigned char x = 0;

  // loop to the and of string
  while ( *dataPtr ) {

    if(inv) {
      LCDChrXYInverse(x, row, (*dataPtr));
    }
    else {
      LCDChrXY( x, row, (*dataPtr));
    }
    x++;
    dataPtr++;
  }
}



