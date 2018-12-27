#include "password.h"
#include "i2C_bsp.h"
#include "global_var.h"
#include <rtthread.h>

extern sys_reg_st					g_sys; 	


void init_work_mode(void)
{
		I2C_EE_BufRead((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
	
		if((g_sys.status.sys_work_mode.work_mode!=WORK_MODE_FSM_OPEN)&&(g_sys.status.sys_work_mode.work_mode!=WORK_MODE_FSM_MANAGE)&&
				(g_sys.status.sys_work_mode.work_mode!=WORK_MODE_FSM_LOCK)&&(g_sys.status.sys_work_mode.work_mode!=WORK_MODE_FSM_LIMIT))
		{
			   g_sys.status.sys_work_mode.work_mode = WORK_MODE_FSM_OPEN;
			  
		}
		
//		rt_kprintf("g_sys.status.sys_work_mode.work_mode = %d\n",g_sys.status.sys_work_mode.work_mode);
}



void runtime(void)
{
	static uint16_t second = 0;
	
	if(second++ >3600)
	{
			second = 0;
			g_sys.status.sys_work_mode.runing_hour++;
			//save time to eeprom
			I2C_EE_BufWrite((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
	}
	if(g_sys.status.sys_work_mode.runing_hour >= 24)
	{
			g_sys.status.sys_work_mode.runing_hour = 0 ;
			g_sys.status.sys_work_mode.runing_time++;
			//save time to eeprom  
			I2C_EE_BufWrite((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
	}
	
	
}

void work_mode_manage(void)
{	
		  static uint16_t work_mode;
		

			switch(work_mode)
			{
				case WORK_MODE_FSM_OPEN:
						if(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_MANAGE)
						{
							  g_sys.config.general.power_mode = 0;
						}
						else if(g_sys.status.sys_work_mode.work_mode ==WORK_MODE_FSM_LOCK)
						{
							  g_sys.config.general.power_mode = 0;
						}
						else
						{
								;
						}
					
				break;
				case WORK_MODE_FSM_MANAGE:
						 g_sys.config.general.power_mode = 0;	
						if(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_OPEN)
						{
								// power_mode open or close determined by eeprom;
								eeprom_compare_read(0,&g_sys.config.general.power_mode);
						}
				break;
				case WORK_MODE_FSM_LOCK:
						 g_sys.config.general.power_mode = 0;
						if(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_LIMIT)
						{
								// power_mode open or close determined by eeprom;
								eeprom_compare_read(0,&g_sys.config.general.power_mode);
								g_sys.status.sys_work_mode.runing_time = 0;
								g_sys.status.sys_work_mode.runing_hour = 0;
								//save_runing_time
								I2C_EE_BufWrite((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
						}
				break;
				case WORK_MODE_FSM_LIMIT:
						 //runing time ++
							runtime();
						 //runing time >limit time jump to lock
						if(g_sys.status.sys_work_mode.runing_time >= g_sys.status.sys_work_mode.limit_time)
						{
									g_sys.status.sys_work_mode.work_mode =  WORK_MODE_FSM_LOCK; 
						}
						else if(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_OPEN)
						{
								// power_mode open or close determined by eeprom;
								eeprom_compare_read(0,&g_sys.config.general.power_mode);
						}
						else
						{
							 ;
						}
						
				break;
				default:
					   ;
				break;
		}
		work_mode = g_sys.status.sys_work_mode.work_mode;
}

uint8_t passward_compare(uint8_t *st1,uint8_t *st2, uint8_t len)
{
		uint8_t i,req;
	
		req = 1;
		for(i=0;i<len;i++)
		{
				if(*(st1+i) != *(st2+i))
				{
						req = 0;
						break;
				}
		}
		return(req);
}

uint8_t cpad_work_mode(uint8_t work_mode,uint16_t day_limit)
{
	uint8_t req;
	req =1;
	
	if(day_limit < 1000)
	{
			g_sys.status.sys_work_mode.work_mode  = work_mode;
			g_sys.status.sys_work_mode.limit_time = day_limit;
			g_sys.status.sys_work_mode.runing_time =0; 
			g_sys.status.sys_work_mode.runing_hour = 0;
	}
	else
	{
		req = 0;
	}

	//write to EEPROM 
	if(req == 1)
	{
			I2C_EE_BufWrite((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
	}
	return(req);
}

uint8_t write_passward (uint8_t*password, uint8_t work_mode,uint16 limit_time)
{
	  uint8_t req ,i;
		req = 1;
		if(limit_time < 1000)
		{
				g_sys.status.sys_work_mode.work_mode  = work_mode;
				for(i=0;i<4;i++)
				{
						g_sys.status.sys_work_mode.pass_word[i] = *(password + i);
				}
				g_sys.status.sys_work_mode.limit_time = limit_time;
				g_sys.status.sys_work_mode.runing_time =0;
				g_sys.status.sys_work_mode.runing_hour =0;
		}
		else
		{
			req = 0;
		}
		if(req == 1)
		{
				I2C_EE_BufWrite((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
		}
		return(req);
}


