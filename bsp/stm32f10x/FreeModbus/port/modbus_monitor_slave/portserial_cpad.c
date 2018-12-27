/*
 * FreeModbus Libary: STM32 Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"
#include <rtthread.h>
/* ----------------------- Modbus includes ----------------------------------*/
#include "fifo.h"
#include "mbport_cpad.h"

/* ----------------------- Start implementation -----------------------------*/
static fifo8_cb_td mnt_tx_fifo;

uint8_t Monitor_isr_flag;
extern cpad_slave_st  cpad_slave_inst;

void cpad_MBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
		eMBParity eParity)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_DeInit(UART5);
	//======================时钟初始化=======================================
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD ,	ENABLE);
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_UART5,ENABLE);
	
	//======================IO初始化=========================================	
	//UART5_TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//UART5_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//配置485发送和接收模式
	//TODO   暂时先写B13 等之后组网测试时再修改
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//======================串口初始化=======================================
	USART_InitStructure.USART_BaudRate = ulBaudRate;
	//设置校验模式
	switch (eParity)
	{
	case MB_PAR_NONE: //无校验
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	case MB_PAR_ODD: //奇校验
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		break;
	case MB_PAR_EVEN: //偶校验
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		break;
	default:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	}

	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART5, &USART_InitStructure);
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5, ENABLE);
	
	


	

	//=====================中断初始化======================================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	CPAD_SLAVE_RS485_RECEIVE_MODE;
	fifo8_init(&mnt_tx_fifo,1,MNT_RX_LEN);

}




void cpad_xMBPortSerialPutByte(uint8_t *ucbyte, uint16_t len)
{
		uint8_t tx_data;
		uint16_t i;
	
		for(i=0;i<len;i++)
		{
			fifo8_push(&mnt_tx_fifo,ucbyte);
			ucbyte++;
		}
	//	rt_kprintf("MonitorxMBPortSerialPutByte\n");

		USART_ClearFlag(UART5, USART_FLAG_TC);
		CPAD_SLAVE_RS485_SEND_MODE;
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
		fifo8_pop(&mnt_tx_fifo,&tx_data);
		USART_SendData(UART5, tx_data);
		USART_ITConfig(UART5, USART_IT_TC, ENABLE);
		
}
/***************************************************************************
 * Function Name  : UART5_IRQHandler
 * Description    : This function handles UART5 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/


void UART5_IRQHandler(void)
{
	uint8_t rec_data,tx_data;
	rt_interrupt_enter();
	//溢出错误
	if (USART_GetFlagStatus(UART5, USART_FLAG_ORE) == SET)
	{
		rec_data = USART_ReceiveData(UART5);
		//USART_ClearFlag(UART5 USART_FLAG_ORE);
	}
	//接收中断
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
			USART_ClearITPendingBit( UART5, USART_IT_RXNE);
			rec_data = USART_ReceiveData(UART5);
			cpad_slave_inst.rx_flag =1;
			if(cpad_slave_inst.rx_ok == 0)
			 {
					switch(cpad_slave_inst.rec_state)
					{
						
						case REC_ADDR_STATE:
						{
							
								if(rec_data == cpad_slave_inst.addr)
								{
									cpad_slave_inst.rec_cnt = 0;
									cpad_slave_inst.rxbuf[cpad_slave_inst.rec_cnt++] = rec_data;	
									cpad_slave_inst.rec_state = REC_DATA_STATE;
								}
								break;
						}
						case REC_DATA_STATE:
						{	
								if(cpad_slave_inst.rec_cnt < MNT_RX_LEN)
								{	
									cpad_slave_inst.rxbuf[cpad_slave_inst.rec_cnt++] = rec_data;	
									if(cpad_slave_inst.rec_cnt >= MNT_CMD_LEN)
									{
											cpad_slave_inst.rx_ok = 1;
											cpad_slave_inst.rec_state = REC_ADDR_STATE;
									}
								}
								break;
						}
						default:
						{
								cpad_slave_inst.rec_state = REC_ADDR_STATE;
								break;
						}
				}
//			}
		}
	}
	//发送中断
	if (USART_GetITStatus(UART5, USART_IT_TC) == SET)
	{
			USART_ClearFlag(UART5, USART_FLAG_TC);
			if(is_fifo8_empty(&mnt_tx_fifo) == 0)
			{
					fifo8_pop(&mnt_tx_fifo,&tx_data);
					USART_SendData(UART5, tx_data);
			}				
			else
			{
					USART_ITConfig(UART5, USART_IT_TC, DISABLE);
					USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
					CPAD_SLAVE_RS485_RECEIVE_MODE;
			}		
			
	}
	
	rt_interrupt_leave();
}
