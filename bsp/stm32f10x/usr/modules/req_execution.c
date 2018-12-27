#include <rtthread.h>
#include "team.h"
#include "calc.h"
#include "sys_conf.h"
#include "local_status.h"
#include "req_execution.h"
#include "alarm_acl_funcs.h"
#include "sys_status.h"
#include "dio_bsp.h"
#include "rtc_bsp.h"
#include <stdlib.h>

enum
{
		PPE_FSM_POWER_ON,
		PPE_FSM_PN_SWITCH,
		PPE_FSM_NO_CONF,
		PPE_FSM_NORMAL,
		PPE_FSM_REVERSE
};
#define INIT_DELAY 	10

#define COMPRESSOR_BITMAP_MASK_POS				0x0006
#define HEATER_BITMAP_MASK_POS						0x0418
#define HUMIDIFIER_BITMAP_MASK_POS				0x0020
#define DEHUMER_BITMAP_MASK_POS						0x0300

#define HUM_CHECKE_INTERVAL     12*3600


enum
{
		RUNING_STATUS_COOLING_BPOS=0,
		RUNING_STATUS_HEATING_BPOS,
		RUNING_STATUS_HUMIDIFYING_BPOS,
		RUNING_STATUS_DEHUMING_BPOS,
};

enum
{
		COMPRESSOR_SIG_HOLD=0,
		COMPRESSOR_SIG_ON,
		COMPRESSOR_SIG_OFF,
		COMPRESSOR_SIG_ERR,
};

enum
{
    WATER_COOLED_SIG_HOLD = 0,
    WATER_COOLED_SIG_ON,
    WATER_COOLED_SIG_OFF,
    WATER_COOLED_SIG_ERR,
};

enum
{
		HEATER_SIG_IDLE=0,
		HEATER_SIG_L1,
		HEATER_SIG_L2,
		HEATER_SIG_L3,
		HEATER_SIG_ERR,
};

enum
{
		HEATER_FSM_IDLE=0,
		HEATER_FSM_L1,
		HEATER_FSM_L2,
		HEATER_FSM_L3,
		HEATER_FSM_ERR,
};


enum
{
		FSM_FAN_IDLE=0,
		FSM_FAN_INIT,
		FSM_FAN_START_UP,
		FSM_FAN_NORM,
		FSM_FAN_SHUT
};

enum
{
		FAN_SIG_IDLE=0,
		FAN_SIG_START,
		FAN_SIG_STOP
};


enum
{
		FAN_TPYE_AC=0,
		FAN_TPYE_EC
};

typedef struct{
		uint16_t time_out;
		uint16_t flush_delay_timer;
		uint16_t hum_fill_cnt;
		uint32_t hum_timer;
		uint32_t check_timer;
		uint8_t  check_fill_flag;
		uint8_t  check_drain_flag;
		uint8_t  check_flag;
		uint16_t warm_time;
}hum_timer;

static hum_timer hum_delay_timer;

void req_bitmap_op(uint8_t component_bpos, uint8_t action)
{
		extern local_reg_st l_sys;
		if(action == 0)
		{
				l_sys.bitmap[BITMAP_REQ] &= ~(0x0001<<component_bpos);
		}
		else
		{
				l_sys.bitmap[BITMAP_REQ] |= (0x0001<<component_bpos);
		}
}

static void req_ao_op(uint8_t component_bpos, int16_t value)
{
		extern local_reg_st l_sys;
    extern sys_reg_st   g_sys;
  
    if((g_sys.config.general.diagnose_mode_en == 0)&&(g_sys.config.general.testing_mode_en == 0))
    {   //normal mode
        switch(component_bpos)
        {
            case (AO_EC_FAN):
            {
                if(1)
                {
                    l_sys.ao_list[component_bpos][BITMAP_REQ]     = value;
                }
                break;
            }
            case (AO_EC_COMPRESSOR):
            {
                if(1)
                {
                    l_sys.ao_list[component_bpos][BITMAP_REQ]     = value;
                }
                break;
            }
            case (AO_WATER_VALVE):
            {
								if(!g_sys.config.water_valve.auto_mode_en)
								{
										l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
								}
								//calc step value
								else if(abs(value - l_sys.ao_list[component_bpos][BITMAP_REQ]) > g_sys.config.water_valve.act_threashold)
								{
										l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
								}
//										rt_kprintf("temp = %d, ao = %d\n", temp, l_sys.ao_list[component_bpos][BITMAP_REQ]);
                break;
            }
            default:
            {
                l_sys.ao_list[component_bpos][BITMAP_REQ]         = value;
                break;
            }				
        }
    }
    else
    {   //dianose or test mode, output directly
        l_sys.ao_list[component_bpos][BITMAP_REQ]     = value;
    }    
}


/**************************************
COMPRESSOR requirement execution function
**************************************/

static uint16_t compressor_signal_gen(int16_t req_temp, int16_t req_hum, uint8_t* comp1_sig, uint8_t* comp2_sig)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;		
		uint8_t comp1_alarm_flag,comp2_alarm_flag;
		uint16_t compressor_count;		
		
		uint32_t	comp1_runtime,comp2_runtime;
		
		if((g_sys.config.general.alarm_bypass_en == 0)&&((l_sys.bitmap[BITMAP_REQ]&(0x0001<<DO_COMP1_BPOS)) != 0)&&((l_sys.bitmap[BITMAP_ALARM]&(0x0001<<DO_COMP1_BPOS)) == 0)&&((l_sys.bitmap[BITMAP_MASK]&(0x0001<<DO_COMP1_BPOS)) != 0))
				comp1_alarm_flag = 1;
		else
				comp1_alarm_flag = 0;

		if((g_sys.config.general.alarm_bypass_en == 0)&&((l_sys.bitmap[BITMAP_REQ]&(0x0001<<DO_COMP2_BPOS)) != 0)&&((l_sys.bitmap[BITMAP_ALARM]&(0x0001<<DO_COMP2_BPOS)) == 0)&&((l_sys.bitmap[BITMAP_MASK]&(0x0001<<DO_COMP2_BPOS)) != 0))
				comp2_alarm_flag = 1;
		else
				comp2_alarm_flag = 0;		

		compressor_count = devinfo_get_compressor_cnt();
		
		comp1_runtime = g_sys.status.run_time[DO_COMP1_BPOS].high;
		comp1_runtime = (comp1_runtime<<4)|(g_sys.status.run_time[DO_COMP1_BPOS].low>>12);
		
		comp2_runtime = g_sys.status.run_time[DO_COMP2_BPOS].high;
		comp2_runtime = (comp2_runtime<<4)|(g_sys.status.run_time[DO_COMP2_BPOS].low>>12);		
		
		if(sys_get_do_sts(DO_FAN_BPOS) == 0)		//fan disabled, emergency shutdown
		{
				*comp1_sig = COMPRESSOR_SIG_ERR;
				*comp2_sig = COMPRESSOR_SIG_ERR;
				l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
				return 0;
		}			
		
		if((sys_get_pwr_sts() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				*comp1_sig = COMPRESSOR_SIG_OFF;
				*comp2_sig = COMPRESSOR_SIG_OFF;
				l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
				return 0;		
		}		
		
		if((sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) != 0)&&(l_sys.l_fsm_state[FAN_FSM_STATE]==FSM_FAN_NORM))
		{		
				if(compressor_count == 1)	//one compressor configured
				{
						if(g_sys.config.compressor.type == 1)		//if it is ec compressor
						{
								if(req_temp >= g_sys.config.compressor.ec_comp_start_req)
								{
										*comp1_sig = COMPRESSOR_SIG_ON;
										*comp2_sig = COMPRESSOR_SIG_OFF;
								}
								else if(req_temp <= 0)
								{
										*comp1_sig = COMPRESSOR_SIG_OFF;
										*comp2_sig = COMPRESSOR_SIG_OFF;
								}
						}
						else
						{
								if((req_temp >= 100)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) == 1))
								{
										*comp1_sig = COMPRESSOR_SIG_ON;
										*comp2_sig = COMPRESSOR_SIG_OFF;
								}
								else if((req_temp <= 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) != 1))
								{
										*comp1_sig = COMPRESSOR_SIG_OFF;
										*comp2_sig = COMPRESSOR_SIG_OFF;
								}
								else
								{
										*comp1_sig = COMPRESSOR_SIG_HOLD;
										*comp2_sig = COMPRESSOR_SIG_OFF;							
								}
						}
						l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
				}
				else if(compressor_count == 2)	//two compressors configured
				{
						if(g_sys.config.compressor.type == 1)		//if it is ec compressor
						{
								
						}
						else
						{
								switch(l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE])
								{
								case(0):
										{
												*comp1_sig = COMPRESSOR_SIG_OFF;
												*comp2_sig = COMPRESSOR_SIG_OFF;
												if((req_temp >= 50)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) != 0))
												{
														if(comp1_runtime < comp2_runtime)
														{
																l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
														}
														else
														{
																l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 2;
														}
												}
												else
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
												}
												break;
										}
								case(1):
										{
												*comp1_sig = COMPRESSOR_SIG_ON;
												*comp2_sig = COMPRESSOR_SIG_OFF;
												if(((req_temp <= 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) == 0))||
														((comp1_alarm_flag&comp2_alarm_flag) != 0))
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
												}
												else if(req_temp >= 100)
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 3;
												}
												else if(((comp1_runtime > comp2_runtime + g_sys.config.compressor.alter_time)&&(g_sys.config.compressor.alter_mode == 1))||		//timing alternation
														(comp1_alarm_flag == 1))				//alarm alternation
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 2;
												}
												else
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
												}
												break;
										}			
								case(2):
										{
												*comp1_sig = COMPRESSOR_SIG_OFF;
												*comp2_sig = COMPRESSOR_SIG_ON;
												if(((req_temp <= 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) == 0))||
														((comp1_alarm_flag&comp2_alarm_flag) != 0))
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
												}
												else if(req_temp >= 100)
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 3;
												}
												else if(((comp2_runtime > comp1_runtime + g_sys.config.compressor.alter_time)&&(g_sys.config.compressor.alter_mode == 1))||		//timing alternation
															(comp2_alarm_flag == 1))				//alarm alternation
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
												}
												else
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 2;
												}												
												break;
										}		
								case(3):
										{
												*comp1_sig = COMPRESSOR_SIG_ON;
												*comp2_sig = COMPRESSOR_SIG_ON;
												if(req_temp < 50)
												{
														if(comp2_runtime > comp1_runtime)
														{
																l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
														}
														else
														{	
																l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 2;
														}														
												}
												else
												{
														l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 3;
												}
												break;
										}
								default:
										{
												*comp1_sig = COMPRESSOR_SIG_OFF;
												*comp2_sig = COMPRESSOR_SIG_OFF;
												l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
												break;
										}
								}
						}						
				}
				else
				{
						*comp1_sig = COMPRESSOR_SIG_OFF;
						*comp2_sig = COMPRESSOR_SIG_OFF;	
						l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;						
				}
		}
		else
		{
				*comp1_sig = COMPRESSOR_SIG_OFF;
				*comp2_sig = COMPRESSOR_SIG_OFF;		
				l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
		}
		return 1;
}


static void compressor_fsm(uint8_t compressor_id, uint8_t signal)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;	
		extern team_local_st team_local_inst;
		
		uint16_t compress_fsm_state;
		
		uint8_t l_fsm_state_id;
		uint8_t do_bpos;
		
		if(compressor_id == 0)
		{
				l_fsm_state_id = COMPRESS1_FSM_STATE;
				do_bpos = DO_COMP1_BPOS;
		}
		else
		{
				l_fsm_state_id = COMPRESS2_FSM_STATE;
				do_bpos = DO_COMP2_BPOS;			
		}
		
		compress_fsm_state = l_sys.l_fsm_state[l_fsm_state_id];
		switch(compress_fsm_state)
		{
				case(COMPRESSOR_FSM_STATE_IDLE):
				{
						if((signal == COMPRESSOR_SIG_ON)&&(l_sys.comp_timeout[do_bpos] == 0))
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
								if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
									(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
								{
										l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.startup_delay;
								}
								else
								{
										l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.startup_delay + ((g_sys.config.team.addr-1)&(0x000f));
								}							
								req_bitmap_op(do_bpos,0);
						}
						else
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
								l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];		
								req_bitmap_op(do_bpos,0);
						}
						break;
				}
				case(COMPRESSOR_FSM_STATE_INIT):
				{
						if(signal != COMPRESSOR_SIG_ON)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
								l_sys.comp_timeout[do_bpos] = 0;
								req_bitmap_op(do_bpos,0);						
						}
						else if((signal == COMPRESSOR_SIG_ON)&&(l_sys.comp_timeout[do_bpos] == 0))
						{
								if(l_sys.comp_startup_interval == 0)
								{
										l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;	
										l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.startup_lowpress_shield;
										req_bitmap_op(do_bpos,1);
										l_sys.comp_startup_interval = g_sys.config.compressor.start_interval;
								}
								else
								{
										l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
										l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];
										req_bitmap_op(do_bpos,0);
								}
						}
						else
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
								l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];		
								req_bitmap_op(do_bpos,0);
						}
						break;
				}				
				case(COMPRESSOR_FSM_STATE_STARTUP):
				{
						if(signal == COMPRESSOR_SIG_ERR)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
								l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
								req_bitmap_op(do_bpos,0);						
						}
						else if(l_sys.comp_timeout[do_bpos] == 0)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;								
								l_sys.comp_timeout[do_bpos] = (g_sys.config.compressor.min_runtime>g_sys.config.compressor.startup_lowpress_shield)? (g_sys.config.compressor.min_runtime - g_sys.config.compressor.startup_lowpress_shield):0;
								req_bitmap_op(do_bpos,1);
						}
						else
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;
								l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];		
								req_bitmap_op(do_bpos,1);
						}
						break;
				}						
				case(COMPRESSOR_FSM_STATE_NORMAL):
				{
						if(signal == COMPRESSOR_SIG_ERR)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
								l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
								req_bitmap_op(do_bpos,0);						
						}
						else if((signal == COMPRESSOR_SIG_OFF)&&(l_sys.comp_timeout[do_bpos] == 0))
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
								l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.stop_delay;
								req_bitmap_op(do_bpos,1);
						}
						else
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
								l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];		
								req_bitmap_op(do_bpos,1);
						}
						break;
				}					
				case(COMPRESSOR_FSM_STATE_SHUTING):
				{
						if(signal == COMPRESSOR_SIG_ERR)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
								l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
								req_bitmap_op(do_bpos,0);						
						}
						else if((signal == COMPRESSOR_SIG_OFF)&&(l_sys.comp_timeout[do_bpos] == 0))
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
								l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
								req_bitmap_op(do_bpos,0);
						}
						else if(signal == COMPRESSOR_SIG_ON)
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
								l_sys.comp_timeout[do_bpos] = 0;
								req_bitmap_op(do_bpos,1);
						}
						else
						{
								l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
								l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];		
								req_bitmap_op(do_bpos,1);
						}
						break;
				}	
				case(COMPRESSOR_FSM_STATE_STOP):
				{						
						l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
						l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
						req_bitmap_op(do_bpos,0);
						break;
				}									
		}
}

static void ec_compressor_output(int16_t req_temp,uint8_t comp_sig)
{
		extern sys_reg_st		g_sys;
		int16_t require;
		
		require = 0;
		
		if(g_sys.config.compressor.type == 0)		//if it is not ec compressor
		{
				if((g_sys.status.dout_bitmap & (0x0001<<DO_COMP1_BPOS)) == 0)
				{
						require = 0;
				}
				else
				{
						require = 100;
				} 
		}
		else
		{
				if((g_sys.status.dout_bitmap & (0x0001<<DO_COMP1_BPOS)) == 0)
				{
						require = 0;
				}
				else
				{
						if(req_temp > g_sys.config.compressor.speed_upper_lim)
						{
								require = g_sys.config.compressor.speed_upper_lim;
						}
						else if(req_temp < g_sys.config.compressor.speed_lower_lim)
						{
								require = g_sys.config.compressor.speed_lower_lim;
						}
						else
						{
								require = req_temp;	
						}
				}		
		}	

		req_ao_op(AO_EC_COMPRESSOR,require);
}



//compressor requirement execution 
static void compressor_req_exe(int16_t req_temp,int16_t req_hum)
{
		uint8_t comp1_sig, comp2_sig;		
		compressor_signal_gen(req_temp,req_hum,&comp1_sig,&comp2_sig);
		compressor_fsm(0,comp1_sig);
		compressor_fsm(1,comp2_sig);
		ec_compressor_output(req_temp,comp1_sig);
}

static uint16_t watervalve_signal_gen(int16_t req_temp, uint8_t* watervalve_sig)
{
    extern sys_reg_st   g_sys;
    extern local_reg_st l_sys;
  
    if(sys_get_do_sts(DO_FAN_BPOS) == 0)    //fan disabled, emergency shutdown
    {
        *watervalve_sig = WATER_COOLED_SIG_ERR;
        return 0;
    }
    
    if((sys_get_pwr_sts() == 0)||
			(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND) || 
			(l_sys.l_fsm_state[FAN_FSM_STATE] != FSM_FAN_NORM)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) == 0))
    {
        *watervalve_sig = WATER_COOLED_SIG_OFF;
        return 0;
    }
    
    if((req_temp > g_sys.config.water_valve.start_req)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) != 0))
    {
        *watervalve_sig = WATER_COOLED_SIG_ON;
    }
    else if(req_temp <= 0)
    {
        *watervalve_sig = WATER_COOLED_SIG_OFF;
    }
    else
    {
        *watervalve_sig = WATER_COOLED_SIG_HOLD;
    }
    
    return 1;
}

static void watervalve_fsm(uint8_t signal)
{
    extern sys_reg_st   g_sys;
    extern local_reg_st l_sys;

    uint16_t watervalve_fsm_state;
  
    watervalve_fsm_state = l_sys.l_fsm_state[WATERVALVE_FSM_STATE];
    
    switch(watervalve_fsm_state)
    {
      case(WATERVALVE_FSM_STATE_STOP):
      {
          if(signal == WATER_COOLED_SIG_ON)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STARTUP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = g_sys.config.water_valve.action_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          break;
      }
      case(WATERVALVE_FSM_STATE_STARTUP):
      {
          if(signal != WATER_COOLED_SIG_ON)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else if(l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] == 0)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_WARMUP;
              l_sys.watervalve_warmup_delay                 = g_sys.config.water_valve.temp_act_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STARTUP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          break;
      }
      case(WATERVALVE_FSM_STATE_WARMUP):
      {
          if(signal == WATER_COOLED_SIG_ERR)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else if(signal == WATER_COOLED_SIG_OFF)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_WARMUP_SHUTING;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = g_sys.config.water_valve.action_delay;
              l_sys.watervalve_warmup_delay                 = l_sys.watervalve_warmup_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else if(l_sys.watervalve_warmup_delay == 0)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_NORMAL;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_WARMUP;
              l_sys.watervalve_warmup_delay                 = l_sys.watervalve_warmup_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          break;
      }
      case(WATERVALVE_FSM_STATE_WARMUP_SHUTING):
      {
          if(signal == WATER_COOLED_SIG_ERR)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else if(signal == WATER_COOLED_SIG_ON)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_WARMUP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              l_sys.watervalve_warmup_delay                 = l_sys.watervalve_warmup_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else if(l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] == 0)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_WARMUP_SHUTING;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              l_sys.watervalve_warmup_delay                 = l_sys.watervalve_warmup_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          break;
      }
      case(WATERVALVE_FSM_STATE_NORMAL):
      {
          if(signal == WATER_COOLED_SIG_ERR)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else if(signal == WATER_COOLED_SIG_OFF)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_SHUTING;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = g_sys.config.water_valve.action_delay;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_NORMAL;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          break;
      }
      case(WATERVALVE_FSM_STATE_SHUTING):
      {
          if(signal == WATER_COOLED_SIG_ERR)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else if(signal == WATER_COOLED_SIG_ON)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_NORMAL;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          else if(l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] == 0)
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_STOP;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = 0;
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 0);
          }
          else
          {
              l_sys.l_fsm_state[WATERVALVE_FSM_STATE]       = WATERVALVE_FSM_STATE_SHUTING;
              l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS] = l_sys.comp_timeout[DO_WATER_VALVE_DUMMY_BPOS];
              req_bitmap_op(DO_WATER_VALVE_DUMMY_BPOS, 1);
          }
          break;
      }
    }
}

static void watervalve_output(int16_t req_temp)
{
    extern sys_reg_st   g_sys;
    extern local_reg_st l_sys;
    uint16_t watervalve_opening;
    uint16_t max, min;
  
    //calc watervalve_opening
    max = g_sys.config.water_valve.max_opening;
    min = g_sys.config.water_valve.min_opening;
    if(req_temp >= 100)
    {
        watervalve_opening = max;
    }
    else if(req_temp <= 0)
    {
        watervalve_opening = min;
    }
    else
    {
        watervalve_opening = req_temp;
    }
    if(watervalve_opening > max)
    {
        watervalve_opening = max;
    }
    if(watervalve_opening < min)
    {
        watervalve_opening = min;
    }
		//手动调节
		if(!g_sys.config.water_valve.auto_mode_en)
		{
				watervalve_opening = g_sys.config.water_valve.set_opening;
		}
    //除湿状态位
    if(sys_get_remap_status(WORK_MODE_STS_REG_NO, DEMHUM_STS_BPOS) != 0)
		{
        watervalve_opening = max;
		}
		else if(sys_get_remap_status(WORK_MODE_STS_REG_NO, HEATING_STS_BPOS) != 0)
		{		//未除湿加热下关闭水阀
				watervalve_opening = 0;
		}
    
    //output
//    if(g_sys.config.water_valve.auto_mode_en)
//    {
        if((g_sys.status.dout_bitmap & (0x0001 << DO_WATER_VALVE_DUMMY_BPOS)))
//				if(l_sys.bitmap[BITMAP_REQ] & (0x0001 << DO_WATER_VALVE_DUMMY_BPOS))
        {
            //auto calc value
            req_ao_op(AO_WATER_VALVE, watervalve_opening);
        }
        else
        {
            req_ao_op(AO_WATER_VALVE, 0);
        }
//    }
//    else
//    {
//        if((g_sys.status.dout_bitmap & (0x0001 << DO_WATER_VALVE_DUMMY_BPOS)))
////				if(l_sys.bitmap[BITMAP_REQ] & (0x0001 << DO_WATER_VALVE_DUMMY_BPOS))
//        {
//            //set value
//            req_ao_op(AO_WATER_VALVE, g_sys.config.water_valve.set_opening);
//        }
//        else
//        {
//            req_ao_op(AO_WATER_VALVE, 0);
//        }
//    }
}

//water valve requirement execution 
static void watervalve_req_exe(int16_t req_temp,int16_t req_hum)
{
		uint8_t watervalve_sig;
    watervalve_signal_gen(req_temp, &watervalve_sig);
    watervalve_fsm(watervalve_sig);
    watervalve_output(req_temp);
}

/**************************************
HEATER requirement execution function
**************************************/

static void heater_req_exe(int16_t req_temp,int16_t req_hum)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;
		uint16_t rh_sts;	
		uint16_t heater_level;
		
		heater_level = devinfo_get_heater_level();
		
		rh_sts = sys_get_do_sts(DO_RH1_BPOS)|(sys_get_do_sts(DO_RH2_BPOS)<<1);
		
		if((l_sys.l_fsm_state[FAN_FSM_STATE] != FSM_FAN_NORM)||(sys_get_pwr_sts() == 0)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) == 0))		//fan disabled or power off, emergency shutdown
		{				
				req_bitmap_op(DO_RH1_BPOS,0);
				req_bitmap_op(DO_RH2_BPOS,0);
				sys_set_remap_status(WORK_MODE_STS_REG_NO,HEATING_STS_BPOS,0);
				
			
//				req_bitmap_op(DO_RH3_BPOS,0);
				return;
		}
		switch(heater_level)
		{
				case (0):
				{
						req_bitmap_op(DO_RH1_BPOS,0);
						req_bitmap_op(DO_RH2_BPOS,0);
//						req_bitmap_op(DO_RH3_BPOS,0);	
						break;
				}				
				case (1):
				{
						if(req_temp<=-100)
						{
								req_bitmap_op(DO_RH1_BPOS,1);	
								req_bitmap_op(DO_RH2_BPOS,0);
//								req_bitmap_op(DO_RH3_BPOS,0);						
						}
						else if(req_temp >= 0)
						{
								req_bitmap_op(DO_RH1_BPOS,0);	
								req_bitmap_op(DO_RH2_BPOS,0);
//								req_bitmap_op(DO_RH3_BPOS,0);						
						}
						else
						{
								;
						}
						break;
				}				
				case (3):
				{
						switch(rh_sts)
						{
								case(HEATER_SIG_IDLE):
								{
										if(req_temp<-33)
										{
												req_bitmap_op(DO_RH1_BPOS,1);
												req_bitmap_op(DO_RH2_BPOS,0);
//											req_bitmap_op(DO_RH3_BPOS,0);		
											
										}
										else
										{
												;
										}
										break;
								}
								case(HEATER_SIG_L1):
								{
										if(req_temp<-66)
										{
												req_bitmap_op(DO_RH1_BPOS,0);
												req_bitmap_op(DO_RH2_BPOS,1);
//												req_bitmap_op(DO_RH3_BPOS,0);								
										}
										else if(req_temp >= 0)
										{
												req_bitmap_op(DO_RH1_BPOS,0);
												req_bitmap_op(DO_RH2_BPOS,0);
//												req_bitmap_op(DO_RH3_BPOS,0);	
										}
										else
										{
												;
										}
										break;
								}				
								case(HEATER_SIG_L2):
								{
										if(req_temp<=-100)
										{
												req_bitmap_op(DO_RH1_BPOS,1);
												req_bitmap_op(DO_RH2_BPOS,1);
//												req_bitmap_op(DO_RH3_BPOS,0);								
										}
										else if(req_temp >= -33)
										{
												req_bitmap_op(DO_RH1_BPOS,1);
												req_bitmap_op(DO_RH2_BPOS,0);
//												req_bitmap_op(DO_RH3_BPOS,0);	
										}
										else
										{
												;
										}
										break;
								}
								case(HEATER_SIG_L3):
								{
										if(req_temp >= -66)
										{
												req_bitmap_op(DO_RH1_BPOS,0);
												req_bitmap_op(DO_RH2_BPOS,1);
//												req_bitmap_op(DO_RH3_BPOS,0);	
										}
										else
										{
												;
										}
										break;
								}
								default:
								{
										req_bitmap_op(DO_RH1_BPOS,0);
										req_bitmap_op(DO_RH2_BPOS,0);
//										req_bitmap_op(DO_RH3_BPOS,0);	
										break;
								}
				
						}
						break;
				}
				default:
				{
						req_bitmap_op(DO_RH1_BPOS,0);	
						req_bitmap_op(DO_RH2_BPOS,0);
//						req_bitmap_op(DO_RH3_BPOS,0);
						break;
				}
		}
		return;
}

/**************************************
HUMIDIFIER requirement execution function
**************************************/
static uint16_t hum_signal_gen(int16_t req_hum)
{
		static uint8_t hum_sig = 0;
		if((req_hum >= 0 )||(sys_get_pwr_sts() == 0)||(sys_get_do_sts(DO_FAN_BPOS) == 0))
		{
				hum_sig=0;
				
		}
		else if(( req_hum <= -100 )&&((sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) == 1)))
		{
				hum_sig = 1;				
		}
		else
		{
				hum_sig = hum_sig	;
		}		
		return hum_sig;
}

//加湿器高水位检测
static uint16_t hum_high_level(void)
{
		uint8_t data;
		static uint32_t hi_level = 0;
		extern sys_reg_st g_sys; 	 
		data =g_sys.status.mbm.hum.water_level;
		if(data == 1)
		{
				hi_level = (hi_level|0x01)<<1;
		}
		else
		{
				hi_level = hi_level<<1;
		}	
		if(hi_level)
		{
				return(1);
		}
		else
		{
				return(0);
		}

}
void hum_capacity_calc(void)
{
		extern sys_reg_st			g_sys;
		switch(g_sys.config.humidifier.hum_cap_type)
		{
			case 0:// 3KG
			{
					g_sys.config.humidifier.hum_real_cap = 30;
					break;
			}
			case 1:// 5 KG
			{
					g_sys.config.humidifier.hum_real_cap = 50;
					break;
			}
			case 2:// 8 KG
			{
					g_sys.config.humidifier.hum_real_cap = 80;
					break;
			}
			case 3:// 10 KG
			{
					g_sys.config.humidifier.hum_real_cap = 100;
					break;
			}
			case 4:// 13 KG
			{
					g_sys.config.humidifier.hum_real_cap = 130;
					break;
			}
			case 5:// 15 KG
			{
					g_sys.config.humidifier.hum_real_cap = 150;
					break;
			}
			case 6:// 23KG
			{
					g_sys.config.humidifier.hum_real_cap = 230;
					break;
			}
			case 7:// 42 KG
			{
					g_sys.config.humidifier.hum_real_cap = 420;
					break;
			}
			default:
			{
					g_sys.config.humidifier.hum_real_cap = g_sys.config.humidifier.hum_capacity;
					break;
			}
		}
}
static void hum_checke_timer(void)
{
		if(hum_delay_timer.check_timer > 0)
		{
			hum_delay_timer.check_timer--;
		}
}
static void hum_time_out(void)
{
	if(hum_delay_timer.time_out > 0)
		{
			hum_delay_timer.time_out--;
		}
}

static void humidifier_fsm(uint8_t hum_sig,int16_t target_req_hum)
{
		extern sys_reg_st			g_sys;
		extern local_reg_st		l_sys;
	
		uint16_t req_hum;
		uint16_t flash_current,fill_current,full_current;
//test
//		time_t now;
//		get_local_time(&now);
		
		if((sys_get_pwr_sts() == 0)||(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)||
		(g_sys.config.humidifier.hum_cool_en == 0)||
		(l_sys.l_fsm_state[FAN_FSM_STATE] != FSM_FAN_NORM)||(sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) == 0))	//fan disabled, emergency shutdown
		{
				l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;	
				sys_set_remap_status(WORK_MODE_STS_REG_NO,HUMING_STS_BPOS,0);
				hum_delay_timer.check_timer = 0;
				req_bitmap_op(DO_DRAIN_BPOS,0);
				req_bitmap_op(DO_FILL_BPOS,0);
				req_bitmap_op(DO_HUM_BPOS,0);								
				return;
		}		

		//K
		if(g_sys.config.humidifier.hum_type)
		{
				req_hum = (0-target_req_hum) / 10;
				req_hum = req_hum * 10;
				if(req_hum < g_sys.config.humidifier.min_percentage)
				{
						req_hum =  g_sys.config.humidifier.min_percentage;
				}
				if(req_hum >100)
				{
						req_hum = 100;
				}
		}
    else
		{
				req_hum = 100;
			
		}
		
		//进入加湿条件
		switch(l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE])
		{
				case HUM_FSM_STATE_IDLE:
				{	
					
						//go to check
						if((hum_sig == 1)&&(get_alarm_bitmap(ACL_HUM_LO_LEVEL) == 0)&&(get_alarm_bitmap(ACL_HUM_DEFAULT) == 0)&&(hum_delay_timer.check_timer == 0)) //启动check 模式
						{
							
								hum_delay_timer.check_fill_flag = 1;
								hum_delay_timer.time_out = g_sys.config.alarm[ACL_HUM_DEFAULT].alarm_param;
							
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_CHECK;
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,1);
//							rt_kprintf("go to HUM_FSM_STATE_CHECK = %d\n",now);
						}
						
						//go to warm
						else if((hum_sig == 1)&&(get_alarm_bitmap(ACL_HUM_LO_LEVEL) == 0)&&(get_alarm_bitmap(ACL_HUM_DEFAULT) == 0)&&(hum_delay_timer.check_timer > 0))
						{
								hum_delay_timer.flush_delay_timer = 0;
								hum_delay_timer.hum_fill_cnt = 0;
								hum_delay_timer.hum_timer = 0;
								hum_delay_timer.time_out = g_sys.config.humidifier.drain_timer;
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,1);
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_WARM;
//										rt_kprintf("go to HUM_FSM_STATE_WARM = %d\n",now);
						}
						else
						{
							  hum_checke_timer();
								req_bitmap_op(DO_FILL_BPOS,0);
								req_bitmap_op(DO_HUM_BPOS,0);
								if(hum_high_level() == 1)//开启排水阀
								{
										req_bitmap_op(DO_DRAIN_BPOS,1);
								}
								else//关闭排水阀
								{
										req_bitmap_op(DO_DRAIN_BPOS,0);
								}
						}
						break;
				}
				case HUM_FSM_STATE_CHECK :
				{
							
							if(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1)
							{
										sys_option_di_sts(DI_HUM_DEFAULT_BPOS,1);
										hum_delay_timer.check_timer = 0;
									  l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;
										req_bitmap_op(DO_HUM_BPOS,0);	
										req_bitmap_op(DO_DRAIN_BPOS,0);
										req_bitmap_op(DO_FILL_BPOS,0);
//									rt_kprintf("go to HUM_FSM_STATE_IDLE= %d\n",now);
							}
							else
							{
									hum_time_out();
									if((hum_delay_timer.check_fill_flag ==1)&&(hum_delay_timer.time_out > 0)) //g_sys.config.alarm[ACL_HUM_DEFAULT].alarm_param))
									{
											req_bitmap_op(DO_HUM_BPOS,0);	
										  req_bitmap_op(DO_DRAIN_BPOS,0);
											req_bitmap_op(DO_FILL_BPOS,1);
											if(hum_high_level() == 1)
											{
													hum_delay_timer.check_fill_flag =0;
													hum_delay_timer.time_out = g_sys.config.humidifier.drain_timer;
											}
									}
									else
									{
											if(hum_delay_timer.check_fill_flag ==0)
											{
													if(hum_delay_timer.time_out > 0)//if(hum_delay_timer.time_out < g_sys.config.humidifier.drain_timer)
													{
															 req_bitmap_op(DO_HUM_BPOS,0);	
															 req_bitmap_op(DO_DRAIN_BPOS,1);
															 req_bitmap_op(DO_FILL_BPOS,0);
														   hum_high_level();
													}
													else
													{
																if(hum_high_level() == 0)
																{
																		hum_delay_timer.time_out = g_sys.config.alarm[ACL_HUM_DEFAULT].alarm_param;
																		hum_delay_timer.check_timer = HUM_CHECKE_INTERVAL;
																	
																		hum_delay_timer.flush_delay_timer = 0;
																		hum_delay_timer.hum_fill_cnt = 0;
																		hum_delay_timer.hum_timer = 0;
																	
																		sys_option_di_sts(DI_HUM_DEFAULT_BPOS,0);
																		l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_WARM;
																		req_bitmap_op(DO_HUM_BPOS,1);	
																		req_bitmap_op(DO_DRAIN_BPOS,0);
																		req_bitmap_op(DO_FILL_BPOS,1);
																	
//																	 rt_kprintf("go to HUM_FSM_STATE_WARM= %d\n",now);
																}
																else
																{																	
																		sys_option_di_sts(DI_HUM_DEFAULT_BPOS,1);
																		hum_delay_timer.check_timer = 0;
																		l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;
																		req_bitmap_op(DO_HUM_BPOS,0);	
																		req_bitmap_op(DO_DRAIN_BPOS,0);
																		req_bitmap_op(DO_FILL_BPOS,0);
//																	rt_kprintf("go to HUM_FSM_STATE_IDLE= %d\n",now);
																}
													}
											}
											else
											{
												
												
													sys_option_di_sts(DI_HUM_DEFAULT_BPOS,1);
													hum_delay_timer.check_timer = 0;
													l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;
													req_bitmap_op(DO_HUM_BPOS,0);	
													req_bitmap_op(DO_DRAIN_BPOS,0);
													req_bitmap_op(DO_FILL_BPOS,0);
//												rt_kprintf("go to HUM_FSM_STATE_IDLE= %d\n",now);
											}
											
									}
						}
						break;
				}
				case HUM_FSM_STATE_WARM:
				{
						full_current = g_sys.config.alarm[ACL_HUM_LO_LEVEL].alarm_param*HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap;
						// go to  HUM_STATE_HUM
						if((hum_sig == 0)||(hum_high_level() == 1)||(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1)||
							(hum_delay_timer.time_out==0)||(g_sys.status.mbm.hum.hum_current > full_current))
						{
							
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_HUM;
							//rt_kprintf("go to HUM_FSM_STATE_HUM= %d\n",now);
						}
						else
						{
								hum_delay_timer.hum_timer++;
								hum_checke_timer();
								hum_time_out();
							
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,1);	
								
						}
					
					break;
				}
					
				case HUM_FSM_STATE_HUM:
				{
						//flash_current mA
						flash_current = (20+ 100)*HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap;
						fill_current = g_sys.config.humidifier.current_percentage*HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap*req_hum/100;
						//	 go to HUM_STATE_IDLE
						if((hum_sig == 0)||(get_alarm_bitmap(ACL_HUM_LO_LEVEL) == 1)||(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1))
						{
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;
//								rt_kprintf("go to HUM_FSM_STATE_IDLE= %d\n",now);
						}
						//go to HUM_STATE_FILL
						else if((g_sys.status.mbm.hum.hum_current < fill_current)&&(hum_high_level() == 0))
						{
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,1);
								hum_delay_timer.hum_fill_cnt++;
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_FILL;	
//								rt_kprintf("go to HUM_FSM_STATE_FILL= %d\n",now);
						}				
						//    go to HUM_STATE_FLUSH 
						else if((hum_delay_timer.flush_delay_timer > g_sys.config.humidifier.flush_interval)&&
							((hum_delay_timer.hum_fill_cnt > g_sys.config.humidifier.fill_cnt)||(hum_delay_timer.hum_timer > g_sys.config.humidifier.hum_time)))
						{	
							
								hum_delay_timer.time_out =  g_sys.config.humidifier.flush_time;
								hum_delay_timer.hum_timer = 0;
								hum_delay_timer.hum_fill_cnt = 0;
								hum_delay_timer.flush_delay_timer =0;
								
							
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,1);
								req_bitmap_op(DO_FILL_BPOS,1);
								
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_FLUSH;
//								rt_kprintf("go to HUM_FSM_STATE_FLUSH= %d\n",now);
						}
						else if(( g_sys.status.mbm.hum.hum_current >= flash_current))// go to drain
						{
							
								hum_delay_timer.time_out =  3;
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,1);
								req_bitmap_op(DO_FILL_BPOS,0);
								
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_DRAIN;
						}
						else //hum
						{
									//开启加湿器
								hum_delay_timer.flush_delay_timer++;
								hum_delay_timer.hum_timer++;
								hum_checke_timer();
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);
						}						
						break;
				}
				case HUM_FSM_STATE_FILL:
				{
						full_current = HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap*req_hum;
						// go to  HUM_STATE_HUM
					  if((hum_sig == 0)||(get_alarm_bitmap(ACL_HUM_LO_LEVEL) == 1)||(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1)||
							(hum_high_level() == 1)||(g_sys.status.mbm.hum.hum_current > full_current))
						{
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);
							
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_HUM;

								if((hum_high_level() == 1)&&(g_sys.status.mbm.hum.hum_current < full_current))
								{
										hum_delay_timer.hum_fill_cnt  = 0;
								}
//									rt_kprintf("go to HUM_FSM_STATE_HUM= %d\n",now);
								
						}
						else
						{
								hum_delay_timer.flush_delay_timer++;
								hum_delay_timer.hum_timer++;
								hum_checke_timer();
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,1);
						}
						break;
				}
				
				case HUM_FSM_STATE_DRAIN:
				{
						
						if((hum_sig == 0)||(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1)||(hum_delay_timer.time_out == 0))
						{
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);	
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_HUM;	
						}
						else //drain
						{
								hum_time_out();
								hum_checke_timer();
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,1);
								req_bitmap_op(DO_FILL_BPOS,0);			
//								rt_kprintf("go to HUM_FSM_STATE_FILL= %d\n",now);
						}
						break;
				}
				case HUM_FSM_STATE_FLUSH:
				{						
						// time out goto HUM_STATE_HUM
						if((hum_sig == 0)||(get_alarm_bitmap(ACL_HUM_DEFAULT) == 1)||(hum_delay_timer.time_out == 0))
						{
								req_bitmap_op(DO_HUM_BPOS,1);	
								req_bitmap_op(DO_DRAIN_BPOS,0);
								req_bitmap_op(DO_FILL_BPOS,0);
								l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_HUM;
//								rt_kprintf("go to HUM_FSM_STATE_ADJUST= %d\n",now);
						}
						else
						{
								hum_time_out();
								hum_checke_timer();
							 //open flush	
								req_bitmap_op(DO_HUM_BPOS,0);	
								req_bitmap_op(DO_DRAIN_BPOS,1);
								req_bitmap_op(DO_FILL_BPOS,1);
								//水位高
								if(hum_high_level() == 1)
								{
										req_bitmap_op(DO_FILL_BPOS,0);//关闭注水阀
								}
								else
								{
										req_bitmap_op(DO_FILL_BPOS,1);//开启注水阀
								}	
						}
						break;
				}
				default:
				{
						l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE] = HUM_FSM_STATE_IDLE;	
						hum_delay_timer.check_timer = 0;
						req_bitmap_op(DO_DRAIN_BPOS,0);
						req_bitmap_op(DO_FILL_BPOS,0);
					//关闭加湿器
						req_bitmap_op(DO_HUM_BPOS,0);
						break;
				}
		}
}

static void humidifier_req_exe(int16_t target_req_hum)
{
		uint8_t hum_sig;
		hum_sig = hum_signal_gen(target_req_hum);
		humidifier_fsm(hum_sig,target_req_hum);
}

/**************************************
DEHUMER requirement execution function
**************************************/
static void dehumer_req_exe(int16_t req_temp, int16_t req_hum)
{
		extern sys_reg_st		g_sys;  
		extern local_reg_st l_sys;
		
		if((g_sys.config.fan.type == FAN_TPYE_AC)&&((g_sys.config.dev_mask.dout&(0x0001<<DO_DEHUM1_BPOS)) == 0))
		{
				req_bitmap_op(DO_DEHUM1_BPOS,0);
				sys_set_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS,0);
				return;
		}
		
		if((sys_get_pwr_sts() == 0)||(g_sys.status.sys_tem_hum.return_air_temp <= g_sys.config.dehumer.stop_dehum_temp)||(l_sys.l_fsm_state[FAN_FSM_STATE] != FSM_FAN_NORM)
			||(sys_get_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS) == 0))
		{
				req_bitmap_op(DO_DEHUM1_BPOS,0);
				sys_set_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS,0);
				return;
		}		
		
		if((g_sys.status.status_remap[WORK_MODE_STS_REG_NO] & (0x0001<<DEMHUM_STS_BPOS)) != 0)		//in dehum mode
		{
				if((req_hum <= 0)||(req_temp >= 150)||(g_sys.status.sys_tem_hum.return_air_temp <= g_sys.config.dehumer.stop_dehum_temp))
				{
						req_bitmap_op(DO_DEHUM1_BPOS,0);
						sys_set_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS,0);
				}
		}
		else
		{
				if((req_temp<125)&&(req_hum >= 100)&&((g_sys.status.sys_tem_hum.return_air_temp > g_sys.config.dehumer.stop_dehum_temp)))
				{
						req_bitmap_op(DO_DEHUM1_BPOS,1);
						sys_set_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS,1);
				}
		}
}

/**************************************
FAN FSM logic process function
**************************************/

static uint8_t fan_signal_gen(void)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;
		uint8_t fan_signal;
		int16_t ao_sig_flag;
		uint16_t i;
		
		ao_sig_flag = 0;
		for(i=(AO_EC_FAN+1);i<AO_MAX_CNT;i++)
		{
				if(l_sys.ao_list[i][BITMAP_REQ] != 0)
				{
						ao_sig_flag =1;
						break;
				}
		}
		
		if(sys_get_pwr_sts() == 1)
		{
				fan_signal = FAN_SIG_START;
		}
		else if((sys_get_pwr_sts() == 0)&&((g_sys.status.dout_bitmap&(~((0x0001<<DO_FAN_BPOS)|(0x0001<<DO_ALARM_BPOS)|(0x0001<<DO_PHASE_P_BPOS)|(0x0001<<DO_PHASE_N_BPOS)))) == 0)&&(ao_sig_flag == 0))
		{
				fan_signal = FAN_SIG_STOP;
		} 
		else
		{
				fan_signal = FAN_SIG_IDLE;
		}
		return fan_signal;
}
/**
  * @brief 	fan output control state FSM execution
	* @param  none
	* @retval none
  */
static void fan_fsm_exe(uint8_t fan_signal)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;
	
		switch(l_sys.l_fsm_state[FAN_FSM_STATE])
		{
				case(FSM_FAN_IDLE):
				{
						if(fan_signal == FAN_SIG_START)
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;
								l_sys.comp_timeout[DO_FAN_BPOS] = g_sys.config.fan.startup_delay;		//assign startup delay to timeout counter
						}
						else
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;	
								l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];	//remains timeout counter
						}
						req_bitmap_op(DO_FAN_BPOS,0);		//disable fan output						
						break;
				}		
				case(FSM_FAN_INIT):
				{
						if(fan_signal != FAN_SIG_START)
						{	
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;	
								req_bitmap_op(DO_FAN_BPOS,0);				//disable fan output
								l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];	//reset timeout counter		
						}
						else
						{
								if(l_sys.comp_timeout[DO_FAN_BPOS] == 0)																
								{
										l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;
									  l_sys.comp_timeout[DO_FAN_BPOS] = g_sys.config.fan.cold_start_delay;
										req_bitmap_op(DO_FAN_BPOS,1);				//enable fan output
								}
								else	//wait until startup delay elapses
								{
										l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;	
										req_bitmap_op(DO_FAN_BPOS,0);	//disable fan output
								}
						}					
						break;
				}	
				case(FSM_FAN_START_UP):
				{
						if(l_sys.comp_timeout[DO_FAN_BPOS] == 0)
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
						}
						else
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;	
						}
						req_bitmap_op(DO_FAN_BPOS,1);				//enable fan output
						l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];	//remain timeout counter
						break;
				}
				case(FSM_FAN_NORM):
				{
						if((fan_signal == FAN_SIG_STOP)&&(l_sys.comp_timeout[DO_FAN_BPOS] == 0))
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;
								l_sys.comp_timeout[DO_FAN_BPOS] = g_sys.config.fan.stop_delay;			//assign startup delay to timeout counter
						}
						else
						{
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;	
								l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];																		//reset timeout counter
						}
						req_bitmap_op(DO_FAN_BPOS,1);																							//disable fan output
						break;					
				}								
				case(FSM_FAN_SHUT):							
				{							
						if(fan_signal == FAN_SIG_START)
						{	
								l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;	
								req_bitmap_op(DO_FAN_BPOS,1);																					//disable fan output
								l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];					//reset timeout counter				
						}
						else
						{
								if(l_sys.comp_timeout[DO_FAN_BPOS] == 0)																
								{
										l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
										req_bitmap_op(DO_FAN_BPOS,0);																			//enable fan output
										l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];																//reset timeout counter					
								}
								else																																		//wait until startup delay elapses
								{
										l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;	
										req_bitmap_op(DO_FAN_BPOS,1);																			//enable fan output
										l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];	//remain timeout counter											
								}
						}	
						break;
				}					
				default:
				{ 
						l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;	
						req_bitmap_op(DO_FAN_BPOS,0);																							//enable fan output
						l_sys.comp_timeout[DO_FAN_BPOS] = 0;																			//reset timeout counter		
						break;
				}			
		}
}


static void ec_fan_output(int16_t req_temp, int16_t req_hum, uint8_t fan_sig)
{
		extern sys_reg_st		g_sys;
		extern local_reg_st 	l_sys;
//		uint16_t bitmap_sense;
		uint16_t status_sense;
		uint16_t require;
		
//		bitmap_sense = 0;
//		status_sense = 0;
//		bitmap_sense = g_sys.status.dout_bitmap & ((0x0001<<DO_COMP1_BPOS) | (0x0001<<DO_COMP2_BPOS) | (0x0001<<DO_RH1_BPOS) | (0x0001<<DO_RH2_BPOS) | (0x0001<<DO_HUM_BPOS));
		status_sense = g_sys.status.status_remap[WORK_MODE_STS_REG_NO] & ((0x0001<<HEATING_STS_BPOS)|(0x0001<<COOLING_STS_BPOS)|(0x0001<<HUMING_STS_BPOS)|(0x0001<<DEMHUM_STS_BPOS));
		require = 0;
		if((g_sys.status.dout_bitmap & (0x0001<<DO_FAN_BPOS)) == 0)
		{
				require = 0;
		}
		else if(g_sys.config.fan.type == FAN_TPYE_EC)
		{
				if(l_sys.l_fsm_state[FAN_FSM_STATE] == FSM_FAN_START_UP)
				{
						require = g_sys.config.fan.set_speed;
				}
				else
				{
						if(g_sys.config.fan.mode == FAN_MODE_FIX)		//fix mode
						{								
								//dehuming mode
								if(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) != 0)
								{
										require = g_sys.config.fan.set_speed*g_sys.config.fan.dehum_ratio/100;
								}
								else if((g_sys.config.fan.noload_down != 0)&&(status_sense == 0))
								{
										require = g_sys.config.fan.min_speed;
								}
								else
								{
										require = g_sys.config.fan.set_speed;
								}
						}
						else if(g_sys.config.fan.mode == FAN_MODE_FLEX)		//flex mode, change fan speed accordingly with compressor or watervalve, to be accomplished
						{
//								if(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER)
//								{
//										require = g_sys.config.fan.set_speed*abs(l_sys.require[TARGET_REQ][T_REQ])/100;
//								}
//								else
//								{
//										require = g_sys.config.fan.set_speed;
//								}								
								
								//dehuming mode
								if(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) != 0)
								{
										if(req_hum<100)
										{
												require = g_sys.config.fan.set_speed*(100 - req_hum)/100;
										}
										else
										{
												require = g_sys.config.fan.min_speed;
										}
								}// heating mode
								else if(sys_get_remap_status(WORK_MODE_STS_REG_NO,HEATING_STS_BPOS) != 0)
								{
										require = g_sys.config.fan.set_speed;
								}// huming mode
								else if(sys_get_remap_status(WORK_MODE_STS_REG_NO,HUMING_STS_BPOS) != 0)
								{
										if(abs(l_sys.require[TARGET_REQ][T_REQ])<abs(l_sys.require[TARGET_REQ][H_REQ]))
										{
												require = g_sys.config.fan.set_speed*abs(l_sys.require[TARGET_REQ][H_REQ])/100;
										}
										else
										{
												require = g_sys.config.fan.set_speed*abs(l_sys.require[TARGET_REQ][T_REQ])/100;
										}
								}
								else
								{
										require = g_sys.config.fan.set_speed*abs(l_sys.require[TARGET_REQ][T_REQ])/100;
								}
						}
						else
						{
								require = g_sys.config.fan.set_speed + l_sys.ec_fan_diff_reg;
						}
				}
				require = require + l_sys.ec_fan_suc_temp;
				require = lim_min_max(g_sys.config.fan.min_speed,g_sys.config.fan.set_speed,require);	
		}
		else if(g_sys.config.fan.type == FAN_TPYE_AC)
		{
				require = 100;
		}
		else
		{
				require = 0;
		}
		
		req_ao_op(AO_EC_FAN,require);
}


/**
  * @brief 	fan output control logic FSM 
	* @param  none
	* @retval none
  */
static void fan_req_exe(int16_t target_req_temp,int16_t target_req_hum)
{
		uint8_t fan_signal;	
		fan_signal = fan_signal_gen();
		fan_fsm_exe(fan_signal);
		ec_fan_output(target_req_temp,target_req_hum,fan_signal);		
}



static void power_phase_switch(void)
{
		extern sys_reg_st		g_sys;
		static uint8_t power_phase_fsm_state = 0;
		static uint16_t switch_delay = 0;
		switch(power_phase_fsm_state)
		{
				case (PPE_FSM_POWER_ON):
				{							
						if((((g_sys.config.dev_mask.dout>>DO_PHASE_P_BPOS) & 0x0001) & ((g_sys.config.dev_mask.dout>>DO_PHASE_N_BPOS) & 0x0001)) == 0)
						{
								power_phase_fsm_state = PPE_FSM_NO_CONF;	
								switch_delay = 0;
								req_bitmap_op(DO_PHASE_P_BPOS,0);
								req_bitmap_op(DO_PHASE_N_BPOS,0);																		
						}
						else
						{
								power_phase_fsm_state = PPE_FSM_PN_SWITCH;	
								switch_delay = INIT_DELAY;
								req_bitmap_op(DO_PHASE_P_BPOS,0);
								req_bitmap_op(DO_PHASE_N_BPOS,0);								
						}
						break;
				}
				case (PPE_FSM_PN_SWITCH):
				{
						if(switch_delay > 0)
						{
								switch_delay--;
						}
						else
						{
								switch_delay = 0;
						}
						
						if(switch_delay == 0)
						{
								if((g_sys.status.mbm.pwr.dev_sts&0x0001) == 0)//power phase error
								{
										power_phase_fsm_state = PPE_FSM_NORMAL;	
										req_bitmap_op(DO_PHASE_P_BPOS,0);
										req_bitmap_op(DO_PHASE_N_BPOS,0);
								}										
								else
								{
										power_phase_fsm_state = PPE_FSM_REVERSE;
										req_bitmap_op(DO_PHASE_P_BPOS,0);
										req_bitmap_op(DO_PHASE_N_BPOS,0);
								}
						}
						break;
				}
				case (PPE_FSM_NO_CONF):
				{
						power_phase_fsm_state = PPE_FSM_NO_CONF;
						switch_delay = 0;
						req_bitmap_op(DO_PHASE_P_BPOS,0);
						req_bitmap_op(DO_PHASE_N_BPOS,0);
						break;
				}				
				case (PPE_FSM_NORMAL):
				{
						power_phase_fsm_state = PPE_FSM_NORMAL;
						switch_delay = 0;
						req_bitmap_op(DO_PHASE_P_BPOS,1);
						req_bitmap_op(DO_PHASE_N_BPOS,0);
						break;
				}
				case (PPE_FSM_REVERSE):
				{
						power_phase_fsm_state = PPE_FSM_REVERSE;
						switch_delay = 0;
						req_bitmap_op(DO_PHASE_P_BPOS,0);
						req_bitmap_op(DO_PHASE_N_BPOS,1);
						break;
				}
				default:
				{
						power_phase_fsm_state = PPE_FSM_NORMAL;
						switch_delay = 0;
						req_bitmap_op(DO_PHASE_P_BPOS,1);
						req_bitmap_op(DO_PHASE_N_BPOS,0);
						break;
				}
		}		
}

enum
{
		FSM_C_COMPRESSOR,
		FSM_C_WATERVALVE
};
static void pumb_execution(void)
{
		//机型判断
		extern sys_reg_st		g_sys;
		if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER)||(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND))
		{
				// 高水位和低水位检测都触发
				if((sys_get_di_sts(DI_COND_HI_LEVEL_BPOS)==1)&&(sys_get_di_sts(DI_COND_LO_LEVEL_BPOS)==1))
				{
							req_bitmap_op(DO_PUMMP_BPOS,1);
				}
				//高水位和低水位检测都解除
				else if((sys_get_di_sts(DI_COND_HI_LEVEL_BPOS)==0)&&(sys_get_di_sts(DI_COND_LO_LEVEL_BPOS)==0))
				{
						 req_bitmap_op(DO_PUMMP_BPOS,0);
				}
				else
				{
						;
				}
		}
}

enum
{
		FSM_HGBP_OFF,
		FSM_HGBP_ON,
};

static void hgbp_exe(int16_t target_req_temp)
{
		extern sys_reg_st		g_sys;
		static uint16_t			hgbp_cd=0;	
		static uint16_t fsm_hgbp_state = FSM_HGBP_OFF;
	
		if(((g_sys.config.dev_mask.dout & ((0x0001)<<DO_HGBP_BPOS)) != 0)&&
				((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)))
		{
				switch(fsm_hgbp_state)
				{
						case FSM_HGBP_OFF:
						{		
								if(((g_sys.status.dout_bitmap&(0x0003<<DO_COMP1_BPOS)) != 0)&&
										(target_req_temp < g_sys.config.hgbp.on_req)&&(target_req_temp>0)&&
										(hgbp_cd == 0))
								{
										req_bitmap_op(DO_HGBP_BPOS,1);
										hgbp_cd = g_sys.config.hgbp.max_on_time;
										fsm_hgbp_state = FSM_HGBP_ON;
								}
								else
								{
										req_bitmap_op(DO_HGBP_BPOS,0);
										if(hgbp_cd > 0)
										{
												hgbp_cd--;
										}
										else
										{
												hgbp_cd = 0;
										}
										fsm_hgbp_state = FSM_HGBP_OFF;
								}	
								break;
						}
						case FSM_HGBP_ON:
						{
								if(((g_sys.status.dout_bitmap&(0x0003<<DO_COMP1_BPOS)) == 0)||
										(target_req_temp > (g_sys.config.hgbp.on_req+g_sys.config.hgbp.hysterisis))||
										(target_req_temp <= 0)||
										(hgbp_cd == 0))
								{
										req_bitmap_op(DO_HGBP_BPOS,0);
										hgbp_cd = g_sys.config.hgbp.min_off_time;
										fsm_hgbp_state = FSM_HGBP_OFF;
								}
								else
								{
										req_bitmap_op(DO_HGBP_BPOS,1);
										if(hgbp_cd > 0)
										{
												hgbp_cd--;
										}
										else
										{
												hgbp_cd = 0;
										}
										fsm_hgbp_state = FSM_HGBP_ON;
								}
								break;
						}
						default:
						{
								req_bitmap_op(DO_HGBP_BPOS,0);
								hgbp_cd = 0;
								fsm_hgbp_state = FSM_HGBP_OFF;								
								break;
						}						
				}
		}
		else
		{
				req_bitmap_op(DO_HGBP_BPOS,0);
				hgbp_cd = 0;
				fsm_hgbp_state = FSM_HGBP_OFF;	
				return;			
		}
}

void req_execution(int16_t target_req_temp,int16_t target_req_hum)
{		
		extern sys_reg_st		g_sys;
		extern local_reg_st l_sys;
		static uint16_t cool_fsm=FSM_C_COMPRESSOR;
		power_phase_switch();
		fan_req_exe(target_req_temp,target_req_hum);
		switch(cool_fsm)
		{
				case(FSM_C_COMPRESSOR):
				{
						if((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER)&&
								(l_sys.l_fsm_state[COMPRESS1_FSM_STATE] == COMPRESSOR_FSM_STATE_IDLE)&&
								(l_sys.l_fsm_state[COMPRESS2_FSM_STATE] == COMPRESSOR_FSM_STATE_IDLE))
						{
								cool_fsm = FSM_C_WATERVALVE;								
						}
						else
						{
								cool_fsm = FSM_C_COMPRESSOR;				
						}
						compressor_req_exe(target_req_temp,target_req_hum);
						break;
				}
				case(FSM_C_WATERVALVE):
				{
						if((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)&&
								(l_sys.l_fsm_state[WATERVALVE_FSM_STATE] == WATERVALVE_FSM_STATE_STOP)&&
								(l_sys.ao_list[AO_WATER_VALVE][BITMAP_REQ] == 0 ))
						{
								cool_fsm = FSM_C_COMPRESSOR;								
						}
						else
						{
								cool_fsm = FSM_C_WATERVALVE;				
						}
						watervalve_req_exe(target_req_temp,target_req_hum);
						break;
				}
				default:
				{
						cool_fsm = FSM_C_COMPRESSOR;
						break;
				}
		}
		heater_req_exe(target_req_temp,target_req_hum);		
		humidifier_req_exe(target_req_hum);		
		dehumer_req_exe(target_req_temp,target_req_hum);
		hgbp_exe(target_req_temp);
		pumb_execution();
}
