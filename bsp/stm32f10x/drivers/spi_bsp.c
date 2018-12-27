#define _SPI_H
#include "stm32f10x.h"
#include <rtthread.h>
#include "spi_bsp.h"
#include "Kits_Delay.h"
#include "stm32f10x_spi.h"


static void SPI_Config(void)
{
#ifdef HARD_SPI
		SPI_InitTypeDef  SPI_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		 
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		
		/* Configure SPI2 pins: NSS, SCK, MISO and MOSI */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

			/*Configure PA.4(NSS)--------------------------------------------*/
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		/* SPI1 configuration */ 
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//????
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8???
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_High=??3,?????? //SPI_CPOL_Low=??0,??????
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Soft;//SPI_NSS_Hard
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_2=18M;//SPI_BaudRatePrescaler_4=9MHz
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//?????????
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		
		SPI_Init(SPI2, &SPI_InitStructure);
			/*Enable SPI1.NSS as a GPIO*/
		SPI_SSOutputCmd(SPI2, ENABLE);

		/* Enable SPI2  */
		SPI_Cmd(SPI2, ENABLE);   
#else
	
		//模拟SPI驱动
		GPIO_InitTypeDef  GPIO_InitStructure; 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	

	  GPIO_InitStructure.GPIO_Pin   = SPI_MOSI_PIN|SPI_SCK_PIN|SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_MOSI_GPIO, &GPIO_InitStructure);	
				
		SPI_SCK_L;
		SPI_MOSI_L;
		SPI_CS_L;
#endif	
		return;
 
}


u8 SPI_Send_Byte(u8 dat)
{
  /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, dat);

  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}


//锁存74HC595
void Latch_74HC595(void)
{

		SPI_CS_L;//设置RCK为低电平，上升沿数据锁存
		Delay_us(5000);
		SPI_CS_H;	
		Delay_us(5000);
		SPI_CS_L;	
		return;
}

/************************************************************************ 
  Function:       Write_74HC595
  Description:    ??n?74HC595?????
  Calls:          HC595_delay;GPIO_ResetBits;GPIO_SetBits;
  Data Accessed:  ?
  Data Updated:   ?
  Input:          
                  HC595x:?????595??,?????74HC595.h?
                 ChipNum: ?????595??????????
  Output:         
                 DataBuf: ?????????
  Return:         ?
  Others:         ????Stm32??????,????72M?????
*************************************************************************/

void Write_74HC595(unsigned char ChipNum,unsigned char *DataBuf)
{
    unsigned char i = 0;
    unsigned char DataBufTmp = 0;
    	
    for(; ChipNum>0; ChipNum--)
    {
        DataBufTmp = *DataBuf;
#ifdef HARD_SPI
    SPI_Send_Byte(DataBufTmp); //
#else
        for(i=0; i<8; i++)
        {   
            SPI_SCK_L; //CLK低   					
            if (DataBufTmp & 0x80)
            {
                SPI_MOSI_H;   //输出1
            }
            else
            {
                SPI_MOSI_L; //输出0
            }        
						Delay_us(5000);
            SPI_SCK_H; //CLK高，上升沿数据移位
            Delay_us(5000);
            
            DataBufTmp <<= 1;
        }
#endif
        DataBuf++;
    }
//    GPIO_SetBits(SPI_595SCLK_GPIO, SPI_595SCLK_PIN);   //CLK高
//    Delay_us(10);
//    GPIO_ResetBits(SPI_595SCLK_GPIO, SPI_595SCLK_PIN); //CLK低
			Latch_74HC595();
			return;
}



void drv_spi_init(void)
{
	SPI_Config();
	return;	

}



