
#include <rtthread.h>

#include "mbport_cpad.h"
#include "string.h"
#include "mbcrc.h"
#include "mb_event_cpad.h"
cpad_slave_st  cpad_slave_inst;

void cpad_MBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
   
    ULONG    usTimerT35_50us;
		cpad_slave_inst.addr = ucSlaveAddress;
		cpad_slave_inst.rec_cnt = 0;
		cpad_slave_inst.rec_state = REC_ADDR_STATE;
		cpad_slave_inst.rx_flag = 0;
		cpad_slave_inst.rx_ok = 0;
		cpad_slave_inst.rx_timeout =0;
		cpad_slave_inst.update_timer_flag = 0;
    /* Modbus RTU uses 8 Databits. */
    cpad_MBPortSerialInit( ucPort, ulBaudRate, 8, eParity );
   
		/* If baudrate > 19200 then we should use the fixed timer values
		 * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
		 */
	
	
		if( ulBaudRate > 19200 )
		{
				usTimerT35_50us = 35;       /* 1800us. */
		}
		else
		{
				/* The timer reload value for a character is given by:
				 *
				 * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
				 *             = 11 * Ticks_per_1s / Baudrate
				 *             = 220000 / Baudrate
				 * The reload for t3.5 is 1.5 times this value and similary
				 * for t3.5.
				 */
				usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
		}
		cpad_xMBPortTimersInit( ( USHORT ) usTimerT35_50us ) ;
      
}



void analysis_protocol(void)
{
		uint8_t cmd;
		uint16_t addr,nreg;
		eMBErrorCode errcode;
		uint16_t sendlen,crc;
	 
	  cmd =  cpad_slave_inst.rxbuf[1];
		addr = (cpad_slave_inst.rxbuf[2]<<8) + cpad_slave_inst.rxbuf[3];
		nreg = (cpad_slave_inst.rxbuf[4]<<8) + cpad_slave_inst.rxbuf[5];
		if(cmd == CPAD_MB_REG_READ)
		{
				errcode = cpad_eMBRegHoldingCB(&cpad_slave_inst.rxbuf[3],addr,nreg,cmd);
				cpad_slave_inst.rxbuf[2] = nreg*2;
				sendlen = cpad_slave_inst.rxbuf[2]+5;
		}
		else
		{
				errcode = cpad_eMBRegHoldingCB(&cpad_slave_inst.rxbuf[4],addr,1,cmd);
				sendlen = 8;
		}
		if(errcode== MB_ENOERR)
		{
			;
		}
		else //erro
		{
			cpad_slave_inst.rxbuf[1]=0x80|cpad_slave_inst.rxbuf[1];
			cpad_slave_inst.rxbuf[2] = errcode;
			sendlen=5;
		}
		//crc
		crc =usMBCRC16(cpad_slave_inst.rxbuf,(sendlen-2));
		cpad_slave_inst.rxbuf[sendlen-1] = crc >>8;
		cpad_slave_inst.rxbuf[sendlen-2] = crc ;
	 //send data
		cpad_xMBPortSerialPutByte(cpad_slave_inst.rxbuf,sendlen)	;
	
		
}
void cpad_MBPoll(void)
{
		uint16_t crc;

		if(cpad_slave_inst.rx_flag)
		{
			cpad_slave_inst.rx_flag = 0;
			//update timer
			cpad_vMBPortTimersEnable();
		}
		
		if((cpad_slave_inst.rx_ok)||(cpad_slave_inst.rx_timeout))
		{
				cpad_vMBPortTimersDisable();
				if(cpad_slave_inst.rx_timeout)
				{
					cpad_slave_inst.rx_ok = 1;
					cpad_slave_inst.rx_timeout =0;
					cpad_slave_inst.rec_state = REC_ADDR_STATE;
				}
			
				//disable timer
				
				if(cpad_slave_inst.rec_cnt>=MNT_RX_MIN)
				{

						crc = usMBCRC16(cpad_slave_inst.rxbuf,cpad_slave_inst.rec_cnt);
						if(crc == 0)
						{
							analysis_protocol();
						}
						else
						{
							rt_kprintf("MNT:check erro =  \n");
						
						}	
				}
				//rx enbale
				cpad_slave_inst.rec_cnt =0;
				cpad_slave_inst.rx_ok =0;	
		}
		
		
}


