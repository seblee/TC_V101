#include <rtthread.h>
#include <components.h>
#include "global_var.h"
#include "Usr_Portserial.h"
#include "Usr_Porttimer.h"
#include "Usr_Protocol.h"


void Comm_PC_Service(void)
{
	if(Comm_PC_inst.rx_flag)
	{
		Comm_PC_inst.rx_flag = 0;
		//update timer
		Comm_PC_PortTimersEnable();
	}
	if((Comm_PC_inst.rx_ok)||(Comm_PC_inst.rx_timeout)||(Comm_PC_inst.State==COM_PC_RCV))
//	if((Comm_PC_inst.rx_ok)||(Comm_PC_inst.rx_timeout))
	{
				g_Var_inst.Test[1]=2;
	    	rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
	    	rt_kprintf("Comm_PC_inst.rx_ok: 0x%02x \n", Comm_PC_inst.rx_ok);
//	    	rt_kprintf("Comm_PC_inst.State: 0x%02x \n", Comm_PC_inst.State);
		//disable timer	
		Comm_PC_PortTimersDisable();
		if(Comm_PC_inst.rx_timeout)
		{
			Comm_PC_inst.rx_timeout =0;
		}
		//接收正确			
		if((Comm_PC_inst.rx_ok)||(Comm_PC_inst.State==COM_PC_RCV))
//		if(Comm_PC_inst.rx_ok)
		{
				g_Var_inst.Test[1]=3;
	    	rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
				if(Analysis_Protocol(&Comm_PC_inst))
				{

				}
		}
		//rx enbale
		Comm_PC_inst.DataCount =0;
		Comm_PC_inst.rx_ok =0;	
		Comm_PC_inst.rec_state = IDLE_STATE;
	}
	return;
}

void Comm_PC_thread_entry(void* parameter)
{
	rt_thread_delay(500);

	Comm_PC_Init(1 , 5, 19200,  COM_PAR_NONE);
	rt_kprintf("Comm_PC_thread\n");
	while(1)
	{
		if(g_Var_inst.Test[1]==0x5A)
		{
			g_Var_inst.Test[1]=1;
	    	rt_kprintf("g_Var_inst.Buff: 0x%02x 0x%02x 0x%02x 0x%02x\n", g_Var_inst.Buff[0],g_Var_inst.Buff[1],g_Var_inst.Buff[2],g_Var_inst.Buff[3]);
	    	rt_kprintf("Comm_PC_inst.Buffer: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
			Comm_PC_inst.Buffer[0],Comm_PC_inst.Buffer[1],Comm_PC_inst.Buffer[2],Comm_PC_inst.Buffer[3],Comm_PC_inst.Buffer[4],Comm_PC_inst.Buffer[5],
			Comm_PC_inst.Buffer[6],Comm_PC_inst.Buffer[7],Comm_PC_inst.Buffer[8],Comm_PC_inst.Buffer[9],Comm_PC_inst.Buffer[10],Comm_PC_inst.Buffer[11]);
		}
		Comm_PC_Service();	 
		rt_thread_delay(500);
	}		
}




