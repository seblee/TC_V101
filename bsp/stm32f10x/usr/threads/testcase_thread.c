#include <rtthread.h>
#include <components.h>
#include "sys_conf.h"
#include "global_var.h"
#include "calc.h"
#include "adc_bsp.h"
#include "i2c_bsp.h"
#include <rtdevice.h>
#include "can_bsp.h"
#include "cpad_bsp.h"
#include "led_bsp.h"
#include "dio_bsp.h"
#include "pwm_bsp.h"
#include "kits/fifo.h"
#include <mb.h>
#include <mb_m.h>
#include "mbconfig.h"
#include "user_mb_app.h"
#include "authentication.h"
#include "local_status.h"
 
void testcase_thread_entry(void* parameter)
{
		RCC_ClocksTypeDef clock_st;
		rt_thread_delay(TESTCASE_THREAD_DELAY);

		RCC_GetClocksFreq(&clock_st);	
//		rt_kprintf("sys_freq:%d\n",clock_st.SYSCLK_Frequency);
		
		while (1)
		{
				rt_thread_delay(1000);
		}
} 

static void show_all(void)
{
    extern local_reg_st 	l_sys;
		extern sys_reg_st			g_sys; 	
	  uint8_t i;
    //FSM
    rt_kprintf("FSM-----TOP   FAN    COMP_SIG     COMP1    COMP2     HUM     WATERVALVE\n");
		rt_kprintf("         %d      %d        %d        %x         %x         %x       %x\n\n",
								l_sys.t_fsm_state,
								l_sys.l_fsm_state[FAN_FSM_STATE],
								l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE],
								l_sys.l_fsm_state[COMPRESS1_FSM_STATE],
								l_sys.l_fsm_state[COMPRESS2_FSM_STATE],
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE],
								l_sys.l_fsm_state[WATERVALVE_FSM_STATE]);
    //BITMAP
    rt_kprintf("Bitmap---REQ    ALARM   MANUAL    FINAL   MASK\n");
    rt_kprintf("         %x      %x      %x         %x      %x\n\n",
               l_sys.bitmap[BITMAP_REQ],
               l_sys.bitmap[BITMAP_ALARM],
               l_sys.bitmap[BITMAP_MANUAL],
               l_sys.bitmap[BITMAP_FINAL],
               l_sys.bitmap[BITMAP_MASK]);
    //AO_LIST
    rt_kprintf("Aolist---FAN    COMP     WV     REV1    REV2\n");
    rt_kprintf("         %d      %d      %d      %d       %d\n\n",
							 g_sys.status.aout[AO_EC_FAN],
							 g_sys.status.aout[AO_EC_COMPRESSOR],
							 g_sys.status.aout[AO_WATER_VALVE],
							 g_sys.status.aout[AO_PREV_1],
							 g_sys.status.aout[AO_PREV_2]); 
	
    //REQ
    rt_kprintf("REQ-----LOCAL_T     LOCAL_H     TEAM_T      TEAM_H      TARGET_T      TARGET_H\n");
    rt_kprintf("        %d          %d          %d        %d        %d          %d\n\n",
               l_sys.require[LOCAL_REQ][T_REQ], l_sys.require[LOCAL_REQ][H_REQ],
               l_sys.require[TEAM_REQ][T_REQ], l_sys.require[TEAM_REQ][H_REQ],
               l_sys.require[TARGET_REQ][T_REQ], l_sys.require[TARGET_REQ][H_REQ]);
    //CD
    rt_kprintf("CD------FAN     COMP1     COMP2     ST_INTERVAL			WATER_VALVE			WATER_WarmUp\n");
    rt_kprintf("        %d        %d        %d          %d				%d			%d\n",
               l_sys.comp_timeout[DO_FAN_BPOS],
               l_sys.comp_timeout[DO_COMP1_BPOS],
               l_sys.comp_timeout[DO_COMP2_BPOS],
               l_sys.comp_startup_interval,
							 l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS],
							 l_sys.watervalve_warmup_delay);
		for(i=0;i<AO_MAX_CNT;i++)
		{
				if(l_sys.ao_list[i][BITMAP_REQ] != 0)
				{		
					rt_kprintf("l_sys.ao_list[%d][BITMAP_REQ] =%d\n",i,l_sys.ao_list[i][BITMAP_REQ]);
				}
		}					 
		
}

static void set_dev_time(uint8_t dev_no,uint16_t hour,uint16_t sec)
{
		if(dev_no <= DO_MAX_CNT)
		{
				if(dev_no == DO_MAX_CNT)
				{
						g_sys.status.sys_work_mode.runing_time =hour;
						g_sys.status.sys_work_mode.runing_hour =sec;
				}
				else
				{
					g_sys.status.run_time[dev_no].high = (hour>>4);
					if(sec>3600)
					{
							sec = 3600;
					}
					g_sys.status.run_time[dev_no].low = sec|((hour&0x000f)<<12);
				}
		}
		else
		{
					rt_kprintf("invalid dev \n");	
		}
}

FINSH_FUNCTION_EXPORT(show_all, show all debug infomation.);

FINSH_FUNCTION_EXPORT(set_dev_time, dev_no hour sec.);

