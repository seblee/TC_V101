 #include <rtthread.h>
#include <components.h>

#include "global_var.h"
#include "Usr_Portserial.h"
#include "Usr_Porttimer.h"
#include "Usr_Protocol.h"

//int Tmpbuf;
void Comm_T_Service(void)
{
		
		switch(Comm_T_inst.State)
		{
			case 0x00:
					if(Comm_T_inst.Rx_ok)//接收到数据
					{
							Comm_T_inst.Rx_ok=0;
							Comm_PC_inst.State=COM_PC_RCV;	//收到返回数据		
							rt_kprintf("Comm_T_inst.Buffer: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
							Comm_T_inst.Buffer[0],Comm_T_inst.Buffer[1],Comm_T_inst.Buffer[2],Comm_T_inst.Buffer[3],Comm_T_inst.Buffer[4],Comm_T_inst.Buffer[5],
							Comm_T_inst.Buffer[6],Comm_T_inst.Buffer[7],Comm_T_inst.Buffer[8],Comm_T_inst.Buffer[9],Comm_T_inst.Buffer[10],Comm_T_inst.Buffer[11]);						
					}
					break;
			case COM_T_RCV:
							Comm_T_inst.State=0;
			rt_kprintf("Comm_T_inst.State: 0x%02x,DataCount:0x%02x,Test[0]:%x\n", Comm_T_inst.State,Comm_T_inst.DataCount,g_Var_inst.Test[0]);
							rt_kprintf("Comm_T_inst.Buffer: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
							Comm_T_inst.Buffer[0],Comm_T_inst.Buffer[1],Comm_T_inst.Buffer[2],Comm_T_inst.Buffer[3],Comm_T_inst.Buffer[4],Comm_T_inst.Buffer[5],
							Comm_T_inst.Buffer[6],Comm_T_inst.Buffer[7],Comm_T_inst.Buffer[8],Comm_T_inst.Buffer[9],Comm_T_inst.Buffer[10],Comm_T_inst.Buffer[11]);		
							Comm_T_PortSerialPutByte(&Comm_T_inst.Buffer[2], Comm_T_inst.DataCount-2);
//					memset(&Comm_T_inst,0x00,sizeof(Comm_T_inst));//清空变量，等待接收
					break;
			default:
					break;			
				
		}
		return;
}
void Comm_T_thread_entry(void* parameter)
{
		rt_thread_delay(500);
//主板		
		Comm_T_Init(1,1,19200,COM_PAR_NONE);
//加湿板
//		g_Var_inst.PC_Address =9;//加湿板
//		g_Var_inst.PC_Baudrate.Value=4800;
//		Comm_T_Init(g_Var_inst.PC_Address,1,g_Var_inst.PC_Baudrate.Value,COM_PAR_NONE);
		rt_kprintf("Comm_T_thread\n");
		while(1)
		{					
				Comm_T_Service();
				rt_thread_delay(500);
		}		
}


