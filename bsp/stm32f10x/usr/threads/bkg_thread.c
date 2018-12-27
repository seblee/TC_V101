#include <rtthread.h>
#include "sys_conf.h"
#include "calc.h"
#include "local_status.h"
#include "team.h"
#include "i2c_bsp.h"
#include "rtc_bsp.h"
#include "event_record.h"
#include "daq.h"
#include "sys_status.h"
#include "watchdog_bsp.h"
#include "password.h"
#include "req_execution.h"
#include "dio_bsp.h"
#include "led_bsp.h"

//static void sys_comp_cooldown(void);
//static void run_time_process(void);
//static void team_tab_cooldown(void);
//static void team_timer(void);
//static void check_team_config(void);
//static void sys_debug_timeout(void);
//static void analog_dummy_out(void);
//static void ec_fan_diff_req(void);
//static void ec_fan_suc_temp(void);
//extern void work_mode_manage(void);

/**
  * @brief 	output control module components cooldown 
	* @param  none
	* @retval none
  */
void bkg_thread_entry(void* parameter)
{				
		//初始化温湿度曲线记录 
		extern sys_reg_st					g_sys; 
	
		rt_thread_delay(500);
//		init_tem_hum_record();
		drv_led_init();
		watchdog_init();
		rt_kprintf("Bkg_thread\n");		
		while(1)
		{
//			team_tab_cooldown();
//			team_timer();
//			sys_comp_cooldown();
//			run_time_process();
//      check_team_config();
//			analog_dummy_out();
//			ec_fan_diff_req();
//			ec_fan_suc_temp();
//			user_eventlog_add();
//			user_alarmlog_add();
//			g_sys.status.sys_tem_hum.return_air_hum = get_current_hum(TARGET_MODE_RETURN);
//			g_sys.status.sys_tem_hum.return_air_temp = get_current_temp(TARGET_MODE_RETURN);	
//			g_sys.status.sys_tem_hum.supply_air_hum = get_current_hum(TARGET_MODE_SUPPLY);
//			g_sys.status.sys_tem_hum.supply_air_temp = get_current_temp(TARGET_MODE_SUPPLY);
//			g_sys.status.sys_tem_hum.remote_air_hum = get_current_hum(TARGET_MODE_REMOTE);
//			g_sys.status.sys_tem_hum.remote_air_temp = get_current_temp(TARGET_MODE_REMOTE);
//				
//			work_mode_manage();
//			add_hum_temp_log();
//			sys_running_mode_update();
//			sys_debug_timeout();
//			//calc_hum_capacity
//			hum_capacity_calc();
			dog();
//			rt_thread_delay(1000);
			led_on(0);
      		rt_thread_delay(500);
      		led_off(0);
      		rt_thread_delay(500);
		}
}

//static void ec_fan_timer_init(void)
//{
//	g_sys.status.flow_diff_timer = g_sys.config.fan.flow_diff_delay;
//	g_sys.status.return_air_status.timer= 0; 
//}
//static void ec_fan_timer_run(void)
//{
//		if(g_sys.status.return_air_status.timer>0)
//		{
//			 g_sys.status.return_air_status.timer --;
//		}
//		else
//		{
//				g_sys.status.return_air_status.timer = 0;
//		}
//		if(g_sys.status.flow_diff_timer>0)
//		{
//			 g_sys.status.flow_diff_timer --;
//		}
//		else
//		{
//				g_sys.status.flow_diff_timer = 0;
//		}
//}
//static void ec_fan_diff_req(void)
//{
//		extern sys_reg_st		g_sys;
//		extern local_reg_st 	l_sys;
//		int16_t ec_fan_diff_tmp;
//		static uint16_t timer_cd = 0;
//
//		if(((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RESERVE2_BPOS))&&(g_sys.config.fan.mode == FAN_MODE_DIFF))				
//		{
//				if(timer_cd != 0)
//				{
//						timer_cd--;
//						ec_fan_diff_tmp = l_sys.ec_fan_diff_reg;
//				}
//				else
//				{
//						timer_cd = g_sys.config.fan.flow_diff_delay;
//						if(g_sys.status.ain[AI_AIR_FLOW_DIFF] > (g_sys.config.fan.set_flow_diff + g_sys.config.fan.flow_diff_deadzone))
//						{
//								ec_fan_diff_tmp = l_sys.ec_fan_diff_reg - g_sys.config.fan.flow_diff_step;
//						}
//						else if(g_sys.status.ain[AI_AIR_FLOW_DIFF] < (g_sys.config.fan.set_flow_diff - g_sys.config.fan.flow_diff_deadzone))
//						{
//								ec_fan_diff_tmp = l_sys.ec_fan_diff_reg + g_sys.config.fan.flow_diff_step;
//						}
//						else
//						{
//								ec_fan_diff_tmp = l_sys.ec_fan_diff_reg;
//						}
//				}
//				
//				l_sys.ec_fan_diff_reg = lim_min_max(-20,20,ec_fan_diff_tmp);
//		}
//		else
//		{
//				l_sys.ec_fan_diff_reg = 0;
//		}
//}
//
//enum
//{
//		FSM_SUC_TEMP_IDLE,
//		FSM_SUC_TEMP_DECRESS
//};
//
//static void ec_fan_suc_temp(void)
//{
//		extern sys_reg_st		g_sys;
//		extern local_reg_st 	l_sys;
//		int16_t suc_temp_reg;
//		static uint16_t timer_cd = 0;	
//		static uint16_t suc_temp_fsm = FSM_SUC_TEMP_IDLE;
//	
//		if(((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_CIOL_NTC1_BPOS))&&((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)||(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)))
//		{
//				if(timer_cd != 0)
//				{
//						timer_cd--;
//						suc_temp_reg = l_sys.ec_fan_suc_temp;
//				}
//				else 
//				{
//						timer_cd = g_sys.config.fan.suc_temp_delay;
//						switch(suc_temp_fsm)
//						{
//								case FSM_SUC_TEMP_IDLE:
//								{
//										if(g_sys.status.ain[AI_NTC5] > (g_sys.config.fan.target_suc_temp+g_sys.config.fan.suc_temp_deadzone))
//										{
//												suc_temp_fsm = FSM_SUC_TEMP_DECRESS;
//												suc_temp_reg = -g_sys.config.fan.suc_temp_step; 
//										}
//										else
//										{
//												suc_temp_fsm = FSM_SUC_TEMP_IDLE;
//												suc_temp_reg = 0; 
//										}
//										break;
//								}
//								case FSM_SUC_TEMP_DECRESS:
//								{
//										if(g_sys.status.ain[AI_NTC5] <= (g_sys.config.fan.target_suc_temp-g_sys.config.fan.suc_temp_deadzone))
//										{
//												suc_temp_fsm = FSM_SUC_TEMP_IDLE;
//												suc_temp_reg = 0; 
//										}
//										else
//										{
//												suc_temp_fsm = FSM_SUC_TEMP_DECRESS;
//												suc_temp_reg = l_sys.ec_fan_suc_temp - g_sys.config.fan.suc_temp_step; 
//										}
//										break;									
//								}
//								default:
//								{
//										suc_temp_fsm = FSM_SUC_TEMP_IDLE;
//										suc_temp_reg = 0; 
//										break;
//								}								
//						}			
//				}
//				l_sys.ec_fan_suc_temp = lim_min_max(-20,20,suc_temp_reg);
//		}
//		else
//		{
//				l_sys.ec_fan_suc_temp = 0;
//		}
//}
//
//
//static void analog_dummy_out(void)
//{
//		extern local_reg_st	l_sys;
//		if((g_sys.config.ext_fan_inst.ext_fan_cnt ==2)&&(g_sys.config.general.cool_type != COOL_TYPE_MODULE_WATER))
//		{
//				if(sys_get_do_sts(DO_COMP1_BPOS) == 1)
//				{
//					l_sys.ao_list[AO_PREV_1][BITMAP_REQ] = g_sys.config.ext_fan_inst.ext_fan1_set_speed;
//				}
//				else
//				{
//						l_sys.ao_list[AO_PREV_1][BITMAP_REQ] = 0;
//				}
//				if(sys_get_do_sts(DO_COMP2_BPOS) == 1)
//				{
//						l_sys.ao_list[AO_PREV_2][BITMAP_REQ] = g_sys.config.ext_fan_inst.ext_fan2_set_speed;
//				}
//				else
//				{
//						l_sys.ao_list[AO_PREV_2][BITMAP_REQ] = 0;
//				}
//		}
//		else if((g_sys.config.ext_fan_inst.ext_fan_cnt ==1)&&(g_sys.config.general.cool_type != COOL_TYPE_MODULE_WATER))
//		{
//				l_sys.ao_list[AO_PREV_2][BITMAP_REQ] = 0;
//				if(sys_get_do_sts(DO_COMP1_BPOS) == 1)
//				{
//						l_sys.ao_list[AO_PREV_1][BITMAP_REQ] = g_sys.config.ext_fan_inst.ext_fan1_set_speed;
//				}
//				else
//				{
//						l_sys.ao_list[AO_PREV_1][BITMAP_REQ] = 0;
//				}
//		}
//		else
//		{
//				l_sys.ao_list[AO_PREV_2][BITMAP_REQ] = 0;
//				l_sys.ao_list[AO_PREV_1][BITMAP_REQ] = 0;
//		}
//}
//
//static void team_tab_cooldown(void)
//{
//		extern team_local_st team_local_inst;
//		uint16_t i;
//		for(i = 0;i < TEAM_MAX_SLAVE_NUM; i++)
//		{
//				if(team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0)
//				{
//						team_local_inst.team_table[i][TEAM_TAB_TIMEOUT]--;
//				}		
//		}
//}
//
//
//static void team_timer(void)
//{
//		extern sys_reg_st		g_sys;
//		extern team_local_st team_local_inst;
//
//		if((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_RUN_BPOS)) != 0)
//		{
//				team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STOP_TIME] = 0;
//				team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_RUN_TIME]++;
//		}
//		else
//		{
//				team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STOP_TIME] ++;
//				team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_RUN_TIME] = 0;				
//		}
//    
//    if(team_local_inst.rotate_timeout < ROTATE_TIMEOUT_CNT)
//    {
//        team_local_inst.rotate_timeout++;
//    }
//		
//		if(team_local_inst.run_sts_cd > 0)
//		{
//				team_local_inst.run_sts_cd--;
//		}
//}
//
//
///**
//  * @brief 	system components runtime counter 
//	* @param  none
//	* @retval none
//  */
//static void time_calc(uint16_t*  sec,uint16_t* h)
//{
//		uint16_t second;
//		uint16_t hour;
//	
//		second = *sec;
//		hour = *h;
//		if((second&0x0fff) >= 3600)
//		{
//				
//				second = (second &0xf000)+0x1000;
//				
//				if(second == 0 )
//				{
//						hour++;
//					  *h = hour;
//				}
//			 *sec = second;
//		}
//}
////HUAWEI
//static void fans_run_time(void)
//{
//		uint8_t i,base_pos;
//		extern sys_reg_st		g_sys; 
//
//		if((g_sys.status.dout_bitmap&(0x0001<<DO_FAN_BPOS)) != 0)
//		{		
//				
//				base_pos= DO_FAN2_DUMMY_BPOS;
//				// fan 2,3,4,5
//				for(i =0 ; i < 4;  i++)
//				{
//						if(get_alarm_bitmap(ACL_FAN_OVERLOAD2 + i) == 0)
//						{
//								g_sys.status.run_time[base_pos+i].low++;
//							
//								time_calc(&g_sys.status.run_time[base_pos+i].low,&g_sys.status.run_time[base_pos+i].high);
//						}
//				}
//			//列间冷冻水
//			if(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER)
//			{
//					if(g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN06_OD_BPOS))
//					{
//							if(get_alarm_bitmap(ACL_FAN_OVERLOAD6 ) == 0)
//							{
//									g_sys.status.run_time[DO_FAN6_DUMMY_BPOS].low++;
//									time_calc(&g_sys.status.run_time[DO_FAN6_DUMMY_BPOS].low,&g_sys.status.run_time[DO_FAN6_DUMMY_BPOS].high);
//							}
//							
//					}
//					if(g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN07_OD_BPOS))
//					{
//							if(get_alarm_bitmap(ACL_FAN_OVERLOAD7 ) == 0)
//							{
//									g_sys.status.run_time[DO_FAN7_DUMMY_BPOS].low++;
//									time_calc(&g_sys.status.run_time[DO_FAN7_DUMMY_BPOS].low,&g_sys.status.run_time[DO_FAN7_DUMMY_BPOS].high);
//							}		
//					}
//					if(g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN08_OD_BPOS))
//					{
//								if(get_alarm_bitmap(ACL_FAN_OVERLOAD8 ) == 0)
//								{
//										g_sys.status.run_time[DO_FAN8_DUMMY_BPOS].low++;
//										time_calc(&g_sys.status.run_time[DO_FAN8_DUMMY_BPOS].low,&g_sys.status.run_time[DO_FAN8_DUMMY_BPOS].high);
//								}		
//							
//					}
//			}		
//		}
//}
//
//	
//static void run_time_process(void)
//{
//		extern sys_reg_st		g_sys; 
//    extern team_local_st team_local_inst;
//		time_t now;
//		uint16_t i;
//		
//		for(i=0; i<DO_FILLTER_DUMMY_BPOS; i++)
//		{
//				if((g_sys.status.dout_bitmap&(0x0001<<i)) != 0)
//				{
//						g_sys.status.run_time[i].low++;
//						
//						time_calc(&g_sys.status.run_time[i].low,&g_sys.status.run_time[i].high);
//				}				
//		}
//		//过滤网运行时间累计
//		if((g_sys.status.dout_bitmap&(0x0001<<DO_FAN_BPOS)) != 0)
//		{	
//				g_sys.status.run_time[DO_FILLTER_DUMMY_BPOS].low++;
//				time_calc(&g_sys.status.run_time[DO_FILLTER_DUMMY_BPOS].low,&g_sys.status.run_time[DO_FILLTER_DUMMY_BPOS].high);
//		}
//		// HAUWEI 风机运行时间扩展
//		fans_run_time();
//		get_local_time(&now);
//		if((now&0x0000007f) == 0x7f)
//		{
//				I2C_EE_BufWrite((uint8_t *)&g_sys.status.run_time,STS_REG_EE1_ADDR,sizeof(g_sys.status.run_time));		//when, fan is working update eeprom every minite		
//		}
//    // bitmapset backup
//    if((now & 0x0000007f) == 0x7f)
//    {
//        I2C_EE_BufWrite((uint8_t *)&team_local_inst.team_bitmap[TEAM_BITMAP_FINAL_OUT],STS_REG_EE2_ADDR,sizeof(team_local_inst.team_bitmap[TEAM_BITMAP_FINAL_OUT]));
//    }
//}
//
//static void sys_comp_cooldown(void)
//{
//		extern local_reg_st	l_sys;
//		uint16_t i;
//
//		for(i=0;i<DO_MAX_CNT;i++)
//		{
//				if(l_sys.comp_timeout[i]>0)
//				{	
//						l_sys.comp_timeout[i]--;
//				}	
//				else
//				{
//						l_sys.comp_timeout[i]=0;
//				}
//		}	
//		
//		if(l_sys.comp_startup_interval > 0)
//		{
//				l_sys.comp_startup_interval--;
//		}
//		else
//		{
//				l_sys.comp_startup_interval = 0;
//		}
//		
//		if(l_sys.watervalve_warmup_delay > 0)
//		{
//				l_sys.watervalve_warmup_delay--;
//		}
//		else
//		{
//				l_sys.watervalve_warmup_delay = 0;
//		}
//}
//
//
//static void check_team_config(void)
//{
//    extern sys_reg_st     g_sys;
//    extern team_local_st  team_local_inst;
//    static uint8_t check_timeout;
//    if((team_local_inst.team_fsm == TEAM_FSM_MASTER) && (g_sys.config.team.addr == 1))
//    {
//        if(check_timeout > CHECK_TEAM_CONFIG_TIMEOUT)
//        {
//            if((team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND]        != g_sys.config.algorithm.temp_deadband)    ||
//               (team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION]       != g_sys.config.algorithm.temp_precision)   ||
//               ( check_target_temp_hum())||
//               (team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND]         != g_sys.config.algorithm.hum_deadband)     ||
//               (team_local_inst.team_config[TEAM_CONF_HUM_PRECISION]        != g_sys.config.algorithm.hum_precision)    ||
//               (team_local_inst.team_config[TEAM_CONF_ALARM_HI_TEMP_PARAM]  != g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param)  ||
//               (team_local_inst.team_config[TEAM_CONF_ALARM_LO_TEMP_PARAM]  != g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param)  ||
//               (team_local_inst.team_config[TEAM_CONF_ALARM_HI_HUM_PARAM]   != g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param)   ||
//               (team_local_inst.team_config[TEAM_CONF_ALARM_LO_HUM_PARAM]   != g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param))
//            {
////                rt_kprintf("%x %x %x %x %x %x %x %x %x %x\n", g_sys.config.algorithm.temp_deadband,
////                                                              g_sys.config.algorithm.temp_precision,
////                                                              (g_sys.config.algorithm.ctrl_target_mode == 0)? g_sys.config.algorithm.return_air_temp:g_sys.config.algorithm.supply_air_temp,
////                                                              g_sys.config.algorithm.hum_deadband,
////                                                              g_sys.config.algorithm.hum_precision,
////                                                              (g_sys.config.algorithm.ctrl_target_mode == 0)? g_sys.config.algorithm.return_air_hum:g_sys.config.algorithm.supply_air_hum,
////                                                              g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param,
////                                                              g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param,
////                                                              g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param,
////                                                              g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param);
////                rt_kprintf("%x %x %x %x %x %x %x %x %x %x\n", team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND],
////                                                              team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION],
////                                                              team_local_inst.team_config[TEAM_CONF_TEMP_SETVAL],
////                                                              team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND],
////                                                              team_local_inst.team_config[TEAM_CONF_HUM_PRECISION],
////                                                              team_local_inst.team_config[TEAM_CONF_HUM_SETVAL],
////                                                              team_local_inst.team_config[TEAM_CONF_ALARM_HI_TEMP_PARAM],
////                                                              team_local_inst.team_config[TEAM_CONF_ALARM_LO_TEMP_PARAM],
////                                                              team_local_inst.team_config[TEAM_CONF_ALARM_HI_HUM_PARAM],
////                                                              team_local_inst.team_config[TEAM_CONF_ALARM_LO_HUM_PARAM]);
//              
//                team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND]        = g_sys.config.algorithm.temp_deadband;
//                team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION]       = g_sys.config.algorithm.temp_precision;
//                team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND]         = g_sys.config.algorithm.hum_deadband;
//                team_local_inst.team_config[TEAM_CONF_HUM_PRECISION]        = g_sys.config.algorithm.hum_precision;
//                team_local_inst.team_config[TEAM_CONF_ALARM_HI_TEMP_PARAM]  = g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param;
//                team_local_inst.team_config[TEAM_CONF_ALARM_LO_TEMP_PARAM]  = g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param;
//                team_local_inst.team_config[TEAM_CONF_ALARM_HI_HUM_PARAM]   = g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param;
//                team_local_inst.team_config[TEAM_CONF_ALARM_LO_HUM_PARAM]   = g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param;
//                team_clear_confd();
//                rt_kprintf("config has changed!\n");
//            }
//            check_timeout = 0;
//        }
//        check_timeout++;
//    }
//    if(team_local_inst.master_send_param_protect > 0)
//    {
//        team_local_inst.master_send_param_protect--;
//    }
//}

//static void sys_debug_timeout(void)
//{
//		extern local_reg_st	l_sys;
//		if(l_sys.debug_tiemout == DEBUG_TIMEOUT_NA)
//		{
//				return;
//		}
//		else if(l_sys.debug_tiemout > 0)
//		{
//				l_sys.debug_tiemout --;
//		}
//		else
//		{
//				l_sys.debug_flag = DEBUG_OFF_FLAG;
//				l_sys.debug_tiemout = 0;
//		}
//}

