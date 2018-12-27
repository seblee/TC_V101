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

//�ڴ浽modbus��ӳ�����
//Ԫ��λ�ö�ӦModeBus  Э��ջ��usSRegHoldBufλ��
//Ԫ��ֵ��Ӧconf_reg_map_inst���ڴ����ݵ�λ�á�

const reg_table_st modebus_slave_reg_table[]={
{0,0},//���ػ�
{0,26},//�趨�ط��¶�
{0,28},//�趨�ط�ʪ��
{1,25},//ϵͳ״̬��
{1,61},//�ط��¶Ȳ���ֵ
{1,62},//�ط�ʪ�Ȳ���ֵ
{1,32},//�澯0
{1,33},//�澯1
{1,34},//�澯2
{1,35},//�澯3
{1,36},//�澯4
{1,37},//�澯5
{0,22},//�¶ȿ��Ʒ�ʽ P or PID
{0,23},//ʪ�ȿ��Ʒ�ʽ
{0,24},//ģʽ����Ŀ�� 
//fan����ʱ��
{1,120},
{1,121},
//ѹ����1����ʱ��
{1,122},
{1,123},
//ѹ����2����ʱ��
{1,124},
{1,125},
{0,25},//�趨�ͷ��¶�
{0,27},//�趨�ͷ�ʪ��
{1,59},//�ͷ��¶Ȳ���ֵ
{1,60},//�ͷ�ʪ�Ȳ���ֵ
{1,65},//��ʪ����
{1,64},//�絼��
{1,117},//�ɽڵ�����״̬0
{1,119},//�ɽ�����״̬
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

//******************************���ּĴ����ص�����**********************************
//��������: eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
//��    �������ּĴ�����صĹ��ܣ�������������д������д��
//��ڲ�����pucRegBuffer : �����Ҫ�����û��Ĵ�����ֵ���������������ָ���µļĴ�����ֵ��
//                         ���Э��ջ��֪����ǰ����ֵ���ص��������뽫��ǰֵд�����������
//			usAddress    : �Ĵ�������ʼ��ַ��
//			usNRegs      : �Ĵ�������
//          eMode        : ����ò���ΪeMBRegisterMode::MB_REG_WRITE���û���Ӧ����ֵ����pucRegBuffer�еõ����¡�
//                         ����ò���ΪeMBRegisterMode::MB_REG_READ���û���Ҫ����ǰ��Ӧ�����ݴ洢��pucRegBuffer��
//���ڲ�����eMBErrorCode : ������������صĴ�����
//��    ע��Editor��Armink 2010-10-31    Company: BXXJS
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
	usAddress--;//FreeModbus���ܺ������Ѿ���1��Ϊ��֤�뻺�����׵�ַһ�£��ʼ�1
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
				
						//������д��Χ�����ж�
								if (pt[iRegIndex].reg_type == 0)
								{
									//д��ֲ�߽��ж�
										Writ_Value=0;
										Writ_Value+=(*pucRegBuffer) << 8;
										Writ_Value+=*(pucRegBuffer+1);
										//max min
										
										if((Writ_Value<=conf_reg_map_inst[pt[iRegIndex].reg_addr].max)&&(Writ_Value>=conf_reg_map_inst[pt[iRegIndex].reg_addr].min)&& 
											(pt[iRegIndex].reg_type == 0))
										{
										
												//д�뱣�ּĴ�����ͬʱ���µ��ڴ��flash����
												// д��Ĵ�����EEPROM�С�
												
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



void mbs_sts_update(void)// ���±��ر�����Э��ջ�Ĵ�����
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