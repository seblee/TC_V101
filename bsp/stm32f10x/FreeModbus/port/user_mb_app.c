/*
 * FreeModbus Libary: user callback functions and buffer define in slave mode
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
 * File: $Id: user_mb_app.c,v 1.60 2013/11/23 11:49:05 Armink $
 */
#include "user_mb_app.h"

#include "global_var.h"
#include "event_record.h"
/*------------------------Slave mode use these variables----------------------*/
//Slave mode:HoldingRegister variables



	


USHORT   usSRegHoldStart                              = S_REG_HOLDING_START;
USHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]           ;
typedef struct 
{
	uint8_t reg_type; //0=config_reg;1 =status_reg;
	uint16_t reg_addr;
	uint8_t reg_w_r; //3 =write&read,2=read_only,3=write_only
}reg_table_st;

//内存到modbus的映射表。
//元素位置对应ModeBus  协议栈中usSRegHoldBuf位置
//元素值对应conf_reg_map_inst，内存数据的位置。

const reg_table_st modebus_slave_reg_table[]={
{0,0},//开关机
{0,26},//设定回风温度
{0,28},//设定回风湿度
{1,25},//系统状态字
{1,61},//回风温度测量值
{1,62},//回风湿度测量值
{1,32},//告警0
{1,33},//告警1
{1,34},//告警2
{1,35},//告警3
{1,36},//告警4
{1,37},//告警5
{0,22},//温度控制方式 P or PID
{0,23},//湿度控制方式
{0,24},//模式控制目标 
//fan运行时间
{1,120},
{1,121},
//压缩机1运行时间
{1,122},
{1,123},
//压缩机2运行时间
{1,124},
{1,125},
{0,25},//设定送风温度
{0,27},//设定送风湿度
{1,59},//送风温度测量值
{1,60},//送风湿度测量值
{1,65},//加湿电流
{1,64},//电导率
{1,117},//干节点输入状态0
{1,119},//干结点输出状态
{1,101},//NTC1
{1,102},//NTC2
{1,103},//NTC3
{1,104},//NTC4
{1,105},//NTC5
{1,106},//NTC6
{1,111},//AOUT1
{1,112},//AOUT2
{1,113},//AOUT3
{1,114},//AOUT4
{1,115},//AOUT5

};

enum
{
  BAUD_4800 = 0,
	BAUD_9600,
	BAUD_19200,
	BAUD_38400,
	BAUD_57600,
	BAUD_115200,
};


typedef struct
{
		uint16_t baudrate;
		uint16_t com_addr;
}communication_change_st;
static communication_change_st com_change_inst;
static void change_surv_baudrate(void)
{
	extern sys_reg_st					g_sys; 
	ULONG ulBaudRate;
	if((com_change_inst.baudrate != g_sys.config.general.surv_baudrate)||(g_sys.config.general.surv_addr != com_change_inst.com_addr ))
	{
			com_change_inst.baudrate  =  g_sys.config.general.surv_baudrate;
			com_change_inst.com_addr = g_sys.config.general.surv_addr;
			switch(com_change_inst.baudrate )
			{
				
				case BAUD_4800:
						ulBaudRate = 4800;
						break;
				case BAUD_9600:
						ulBaudRate = 9600;
						break;
				case BAUD_19200:
						ulBaudRate = 19200;
						break;
				case BAUD_38400:
						ulBaudRate = 38400;
						break;
				case BAUD_57600:
						ulBaudRate = 57600;
						break;
				case BAUD_115200:
						ulBaudRate = 115200;
						break;
				default:
						ulBaudRate = 9600;
					break;
			}
			rt_kprintf("change_surv_baudrate = %d",ulBaudRate);
			eMBInit(MB_RTU,(UCHAR)g_sys.config.general.surv_addr, 1, ulBaudRate,  MB_PAR_NONE);
			eMBEnable();
	}
}
void modbus_slave_thread_entry(void* parameter)
{
		extern sys_reg_st		g_sys; 
		eMBErrorCode    eStatus = MB_ENOERR;
	
		rt_thread_delay(MODBUS_SLAVE_THREAD_DELAY);
		eStatus = eMBInit(MB_RTU, g_sys.config.general.surv_addr , 1, 19200,  MB_PAR_NONE);
		com_change_inst.baudrate = BAUD_19200;
		com_change_inst.com_addr = g_sys.config.general.surv_addr;
	
		if(eStatus != MB_ENOERR)
		{
				rt_kprintf("MB_SLAVE init failed\n");
		}
		eStatus = eMBEnable();			
		if(eStatus != MB_ENOERR)
		{
				rt_kprintf("MB_SLAVE enable failed\n");	
		}
		while(1)
		{
				eStatus = eMBPoll();
				if(eStatus != MB_ENOERR)
				{
						rt_kprintf("MB_SLAVE enable failed\n");	
				}	
				mbs_sts_update();	
				change_surv_baudrate();
				rt_thread_delay(10);
		}
}

//******************************保持寄存器回调函数**********************************
//函数定义: eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
//描    述：保持寄存器相关的功能（读、连续读、写、连续写）
//入口参数：pucRegBuffer : 如果需要更新用户寄存器数值，这个缓冲区必须指向新的寄存器数值。
//                         如果协议栈想知道当前的数值，回调函数必须将当前值写入这个缓冲区
//			usAddress    : 寄存器的起始地址。
//			usNRegs      : 寄存器数量
//          eMode        : 如果该参数为eMBRegisterMode::MB_REG_WRITE，用户的应用数值将从pucRegBuffer中得到更新。
//                         如果该参数为eMBRegisterMode::MB_REG_READ，用户需要将当前的应用数据存储在pucRegBuffer中
//出口参数：eMBErrorCode : 这个函数将返回的错误码
//备    注：Editor：Armink 2010-10-31    Company: BXXJS
//**********************************************************************************
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	extern sys_reg_st  g_sys; 
	eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    USHORT *        pusRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

	extern  conf_reg_map_st conf_reg_map_inst[];
	uint16              Writ_Value;
	const reg_table_st *           pt= modebus_slave_reg_table;

	


	pusRegHoldingBuf = usSRegHoldBuf;
	REG_HOLDING_START = S_REG_HOLDING_START;
	REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
	usRegHoldStart = usSRegHoldStart;
	usAddress--;//FreeModbus功能函数中已经加1，为保证与缓冲区首地址一致，故减1
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = usAddress - usRegHoldStart;
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
								*pucRegBuffer++ = ( unsigned char )( pusRegHoldingBuf[iRegIndex] >> 8 );
							  *pucRegBuffer++ = ( unsigned char )( pusRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
						//forbid modbuss option power switch
						if((iRegIndex == 0)&&(g_sys.config.general.power_mode_mb_en ==0))
						{
								eStatus = MB_ENOREG;
								break;//	case MB_REG_WRITE:
						}
            while( usNRegs > 0 )
            {
				
						//超出可写范围报错判断
								if (pt[iRegIndex].reg_type == 0)
								{
									//写入植边界判断
										Writ_Value=0;
										Writ_Value+=(*pucRegBuffer) << 8;
										Writ_Value+=*(pucRegBuffer+1);
										//max min
										
										if((Writ_Value<=conf_reg_map_inst[pt[iRegIndex].reg_addr].max)&&(Writ_Value>=conf_reg_map_inst[pt[iRegIndex].reg_addr].min)&& 
											(pt[iRegIndex].reg_type == 0))
										{
										
												//写入保持寄存器中同时跟新到内存和flash保存
												// 写入寄存器和EEPROM中。
												
												if(reg_map_write(conf_reg_map_inst[pt[iRegIndex].reg_addr].id,&Writ_Value,1,USER_MODEBUS_SLAVE)
													==CPAD_ERR_NOERR)
													{
																pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
																pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
																iRegIndex++;
																usNRegs--;	

													}
													else
													{
														
														eStatus = MB_ENORES;
														break;//	 while( usNRegs > 0 )
													}
											}
											else
											{
												
												eStatus = MB_EINVAL;
												break;// while( usNRegs > 0 )
											}
									
									
								}
								else
								{
										
									eStatus = MB_ENOREG;
										break;//  while( usNRegs > 0 )
								}
					
						
						
				}
			break;
        }
    }
    else
    {
    
        eStatus = MB_ENOREG;
    }
    return eStatus;
}



void mbs_sts_update(void)// 更新本地变量到协议栈寄存器中
{
		char i=0;
		const reg_table_st *           pt = modebus_slave_reg_table;
		extern	conf_reg_map_st conf_reg_map_inst[];
	  extern  sts_reg_map_st status_reg_map_inst[];
		
		for(i=0;i<S_REG_HOLDING_NREGS;i++)
		{
				if(pt[i].reg_type == 0)
				{
						usSRegHoldBuf[i]=*(conf_reg_map_inst[pt[i].reg_addr].reg_ptr);
				}
				else
				{
						usSRegHoldBuf[i]=*(status_reg_map_inst[pt[i].reg_addr].reg_ptr);
				}
		} 
		
	
}
