#include <rtthread.h>
#include <components.h>
#include "sys_conf.h"
#include "local_status.h"
#include "daq.h"
#include "team.h"
#include "require_calc.h"

//int16_t i16TestMeasure[200] =
//{//调试用测量值
//	200,201,202,203,204,205,206,207,208,209,
//	210,211,212,213,214,215,216,217,218,219,
//	220,221,222,223,224,225,226,227,228,229,
//	230,231,232,233,234,235,236,237,238,239,
//	240,241,242,243,244,245,246,247,248,249,
//	250,251,252,253,254,255,256,257,258,259,
//	260,261,262,263,264,265,266,267,268,269,
//	270,271,272,273,274,275,276,277,278,279,
//	280,281,282,283,284,285,286,287,288,289,
//	290,291,292,293,294,295,296,297,298,299,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//  250,250,250,250,250,250,250,250,250,250,
//};

static pid_reg_st pid_reg_inst;

#define P_DATA 20
#define I_DATA 3  //0.6
#define D_DATA 1

//static pid_param_st s_pid_param;

void inc_pid_param_init(void)
{
//    s_pid_param.last_error   = 0; //Error[-1]
//    s_pid_param.prev_error   = 0; //Error[-2]
  
    pid_reg_inst.p_saved    = 0;
    pid_reg_inst.i_saved    = 0;
    pid_reg_inst.req_saved  = 0;
}

//增量式PID控制设计
//static int16_t inc_pid_calc(int16_t next_point)
//{
//    extern sys_reg_st			g_sys;
//    int16_t cur_error, calc_pid; //当前误差
//  
//    s_pid_param.proportion   = P_DATA; //Proportional Const
//    s_pid_param.integral     = g_sys.config.algorithm.temp_integ / 10;//0.6  //Integral Const
//    s_pid_param.derivative   = g_sys.config.algorithm.temp_diff / 10; //1    //Derivative Const
//  
//    if(g_sys.config.algorithm.ctrl_target_mode == 0)
//		{
//				s_pid_param.set_point = g_sys.config.algorithm.return_air_temp;
//		}
//		else
//		{
//				s_pid_param.set_point = g_sys.config.algorithm.supply_air_temp;
//		}

//    cur_error  = s_pid_param.set_point - next_point; //增量计算
//    calc_pid   = s_pid_param.proportion * cur_error //E[k]项
//                + s_pid_param.integral * s_pid_param.last_error //E[k-1]项
//                + s_pid_param.derivative * (s_pid_param.prev_error); //E[k-2]项
//  
//    //存储误差，用于下次计算
//    s_pid_param.prev_error = s_pid_param.last_error;
//    s_pid_param.last_error = cur_error;
//  
//    if(calc_pid > (g_sys.config.algorithm.pid_action_max * 10))
//    {
//        calc_pid = g_sys.config.algorithm.pid_action_max * 10;
//    }
//    else if(calc_pid < (-g_sys.config.algorithm.pid_action_max * 10))
//    {
//        calc_pid = -g_sys.config.algorithm.pid_action_max * 10;
//    }
//    return(calc_pid / 10);                         //返回增量值
//}

static int16_t p_algorithm(int16_t real_value, int16_t set_value, uint16_t deadzone, uint16_t precision)
{
		int16_t require;
		if(real_value > set_value)
		{
				require = ((real_value - set_value - deadzone)*100)/precision;
				if(require<0)
				{
						require = 0;
				}
		}
		else
		{
				require = ((real_value - set_value + deadzone)*100)/precision;
				if(require>0)
				{
						require = 0;
				}
		}	
		return require;
}


static int16_t pid_temp_algorithm(int16_t p_new, int16_t *p_saved, int16_t *i_saved, int16_t *req_saved)
{
		extern sys_reg_st			g_sys;
		int16_t temp_d_param;
		int16_t temp_p_param;
		int16_t temp_req;
		
		temp_p_param = p_new;
		if (temp_p_param > g_sys.config.algorithm.pid_action_max)
		{
			temp_p_param = g_sys.config.algorithm.pid_action_max;
		}
		else if (temp_p_param < -g_sys.config.algorithm.pid_action_max)
		{
			temp_p_param = -g_sys.config.algorithm.pid_action_max;
		}
		
		(*i_saved) = temp_p_param * g_sys.config.algorithm.sample_interval / g_sys.config.algorithm.temp_integ;		

		if ((*i_saved) > g_sys.config.algorithm.pid_action_max)
		{
			(*i_saved) = g_sys.config.algorithm.pid_action_max;
		}
		else if ((*i_saved) < -g_sys.config.algorithm.pid_action_max)
		{
			(*i_saved) = -g_sys.config.algorithm.pid_action_max;
		}
		
		temp_d_param = (temp_p_param - *p_saved)*g_sys.config.algorithm.temp_diff / g_sys.config.algorithm.sample_interval;		

		if (temp_d_param > g_sys.config.algorithm.pid_action_max)
		{
				temp_d_param = g_sys.config.algorithm.pid_action_max;
		}
		else if (temp_d_param < -g_sys.config.algorithm.pid_action_max)
		{
				temp_d_param = -g_sys.config.algorithm.pid_action_max;
		}		
		
		*p_saved = temp_p_param;
		
		temp_req = temp_p_param + *i_saved + temp_d_param;

		if ((temp_req - *req_saved)  > g_sys.config.algorithm.temp_req_out_max)
		{
				temp_req = *req_saved + g_sys.config.algorithm.temp_req_out_max;
		}
		else if ((temp_req - *req_saved)  < -g_sys.config.algorithm.temp_req_out_max)
		{
				temp_req = *req_saved -g_sys.config.algorithm.temp_req_out_max;
		}	

		if (temp_req > 200)
		{
			temp_req = 200;
		}
		else if (temp_req < -200)
		{
			temp_req = -200;
		}		
		
		*req_saved = temp_req;
	
		return temp_req;
}

int16_t team_temp_req_calc(uint8_t type)
{
    extern team_local_st  team_local_inst;
    extern sys_reg_st     g_sys;
    int16_t   ret;
    int16_t   temp_average_req;
//    uint16_t  team_mode;
    uint16_t  total_work_cnt;
    uint16_t  output_cnt, total_cnt;
    uint8_t i;
  
    temp_average_req  = team_temp_average_req_calc();
//    team_mode         = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TMMOD_BPOS) & 0x0003;
    total_work_cnt    =(uint16_t)(team_local_inst.team_config[TEAM_CONF_CNT] & 0xff) - (uint16_t)((team_local_inst.team_config[TEAM_CONF_CNT] >> 8) & 0xff);
    total_cnt         = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
    output_cnt        = 0;
    for(i = 0; i < total_cnt; i++)
    {
        if((team_local_inst.team_bitmap[TEAM_BITMAP_OUTPUT] & (0x0001 << i)) != 0)
        {
            output_cnt++;
        }
    }
    
    switch(type)
    {
        case TOTAL_REQ:
            ret = temp_average_req * total_work_cnt;
            break;
        case AVER_REQ:
            ret = temp_average_req * total_work_cnt / output_cnt;
            break;
        default:
            ret = temp_average_req * total_work_cnt;
          break;
    }
  
    return ret;
}


int16_t team_temp_average_req_calc(void)
{
		extern team_local_st team_local_inst;
		int16_t temp_req;
		int16_t current_temp;
		int16_t set_temp;				
		uint16_t i, calc_cnt;
		uint16_t temp_mode, deadband, precision, total_cnt;
		
		current_temp  = 0;
		temp_mode     = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TMODE_BPOS) & 0x0001;
		deadband      = team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND];
		precision     = team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION];
    total_cnt     = (team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff);
		calc_cnt      = 0;	
		
		set_temp      = (int16_t)team_local_inst.team_config[TEAM_CONF_TEMP_SETVAL];
		
		for(i = 0; i < total_cnt; i++)
		{
				if((team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP] > TEMP_MIN) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP] < TEMP_MAX))
				{
						current_temp += (int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP];
						calc_cnt++;
				}
		}
		if(calc_cnt > 0)
		{
				current_temp = current_temp / calc_cnt;
		}
		else
		{
				current_temp = 0x7fff;
		}		

		
		switch(temp_mode)
		{
				case(P_ALOGORITHM):
				{						
						temp_req = p_algorithm(current_temp,set_temp,deadband,precision);
						break;
				}
				case(PID_ALOGORITHM):
				{						
						temp_req = p_algorithm(current_temp,set_temp,deadband,precision);
            temp_req = pid_temp_algorithm(temp_req, &pid_reg_inst.p_saved, &pid_reg_inst.i_saved, &pid_reg_inst.req_saved);
						break;
				}
				default:
				{
						temp_req = 0;
						break;
				}
		}	
		
		if(current_temp >= 0x7fff)
		{
				temp_req = 0;
		}

		return temp_req;
}

static int16_t abs_hum_calc(int16_t hum_relative)
{
		extern sys_reg_st		g_sys;
		int16_t hum_abs;
		int16_t set_temp;
		int16_t current_temp;
	
		if(g_sys.config.algorithm.ctrl_target_mode == TARGET_MODE_REMOTE)
		{	
				set_temp = g_sys.config.algorithm.remote_air_temp;
				current_temp = g_sys.status.sys_tem_hum.remote_air_temp;
		}		
		else
		{
				set_temp = g_sys.config.algorithm.return_air_temp;
				current_temp = g_sys.status.sys_tem_hum.return_air_temp;			
		}
		
		if(current_temp >= 0x7fff)
		{
				return(0x7fff);
		}
		hum_abs = hum_relative + (current_temp - set_temp - g_sys.config.algorithm.temp_deadband)/2;
		
		return hum_abs;		
}

int16_t team_hum_req_calc(uint8_t type)
{
    extern team_local_st  team_local_inst;
    extern sys_reg_st     g_sys;
    int16_t   ret;
    int16_t   hum_average_req;
//    uint16_t  team_mode;
    uint16_t  total_work_cnt;
    uint16_t  output_cnt, total_cnt;
    uint8_t   i;
  
    hum_average_req   = team_hum_average_req_calc();
//    team_mode         = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TMMOD_BPOS) & 0x0003;
    total_work_cnt    = (uint16_t)(team_local_inst.team_config[TEAM_CONF_CNT] & 0xff) - (uint16_t)((team_local_inst.team_config[TEAM_CONF_CNT] >> 8) & 0xff);
    total_cnt         = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
    output_cnt        = 0;
    for(i = 0; i < total_cnt; i++)
    {
        if((team_local_inst.team_bitmap[TEAM_BITMAP_OUTPUT] & (0x0001 << i)) != 0)
        {
            output_cnt++;
        }
    }
    switch(type)
    {
        case TOTAL_REQ:
            ret = hum_average_req * total_work_cnt;
            break;
        case AVER_REQ:
            ret = hum_average_req * total_work_cnt / output_cnt;
            break;
        default:
            ret = hum_average_req * total_work_cnt;
          break;
    }

    return ret;
}  

int16_t team_hum_average_req_calc(void)
{
		extern team_local_st team_local_inst;
		int16_t hum_req;
		int16_t current_hum;
		int16_t set_hum;				
		uint16_t i, calc_cnt;
		uint16_t hum_mode, deadband, precision, total_cnt;
		
		current_hum   = 0;
		hum_mode      = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_HMODE_BPOS) & 0x0001;
		deadband      = team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND];
		precision     = team_local_inst.team_config[TEAM_CONF_HUM_PRECISION];
    total_cnt     = (team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff);
		calc_cnt      = 0;		
		
		set_hum       = (int16_t)team_local_inst.team_config[TEAM_CONF_HUM_SETVAL];
  
		for(i=0; i < total_cnt; i++)
		{
				if((team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM] > HUM_MIN) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM] < HUM_MAX))
				{
						current_hum += (int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM];
						calc_cnt++;
				}
		}
		if(calc_cnt > 0)
		{
				current_hum = current_hum / calc_cnt;
		}
		else
		{
				current_hum = 0x7fff;
		}

		if(hum_mode == HUM_ABSOLUTE)
		{
				current_hum = abs_hum_calc(current_hum);
		}

		if(current_hum >= 0x7fff)
		{
				hum_req = 0;
		}
		else
		{
				hum_req = p_algorithm(current_hum,set_hum,deadband,precision);
		}	

		return hum_req;
}

static int16_t local_temp_req_calc(void)
{
		extern sys_reg_st		g_sys;
		int16_t temp_req;
		int16_t current_temp;
		int16_t set_temp;		
		
		if(g_sys.config.algorithm.ctrl_target_mode == TARGET_MODE_RETURN)
		{
				set_temp = g_sys.config.algorithm.return_air_temp;
				current_temp = g_sys.status.sys_tem_hum.return_air_temp;	
		}
		else if(g_sys.config.algorithm.ctrl_target_mode == TARGET_MODE_SUPPLY)
		{
				set_temp = g_sys.config.algorithm.supply_air_temp;
				current_temp = g_sys.status.sys_tem_hum.supply_air_temp;
		}				
		else
		{
				set_temp = g_sys.config.algorithm.remote_air_temp;
				current_temp = g_sys.status.sys_tem_hum.remote_air_temp;		
		}
		
		if(current_temp >= 0x7fff)
		{
				return 0;
		}

		
		switch(g_sys.config.algorithm.temp_ctrl_mode)
		{
				case(P_ALOGORITHM):
				{						
						temp_req = p_algorithm(current_temp,set_temp,g_sys.config.algorithm.temp_deadband,g_sys.config.algorithm.temp_precision);
						break;
				}
				case(PID_ALOGORITHM):
				{						
						temp_req = p_algorithm(current_temp,set_temp,g_sys.config.algorithm.temp_deadband,g_sys.config.algorithm.temp_precision);
						temp_req = pid_temp_algorithm(temp_req, &pid_reg_inst.p_saved, &pid_reg_inst.i_saved, &pid_reg_inst.req_saved);
//            temp_req = inc_pid_calc(current_temp);
						break;
				}
				default:
				{
						temp_req = 0;
						break;
				}
		}
		
		return temp_req;				
}



static int16_t local_hum_req_calc(void)
{
		extern sys_reg_st		g_sys;

		int16_t hum_req;
		int16_t current_hum;
		int16_t set_hum;		
		
	
		if(g_sys.config.algorithm.ctrl_target_mode == TARGET_MODE_REMOTE)
		{
				set_hum = g_sys.config.algorithm.remote_air_hum;
		}
		else
		{
				set_hum = g_sys.config.algorithm.return_air_hum;
		}
		
		if(g_sys.config.algorithm.ctrl_target_mode == TARGET_MODE_REMOTE)
		{
				current_hum = g_sys.status.sys_tem_hum.remote_air_hum;
		}
		else
		{
				current_hum = g_sys.status.sys_tem_hum.return_air_hum;	
		}
		


		
		if(current_hum >= 0x7fff)
		{
				return 0;
		}		

		if(g_sys.config.algorithm.hum_ctrl_mode == HUM_ABSOLUTE)
		{
				current_hum = abs_hum_calc(current_hum);
		}
		if(current_hum >= 0x7fff)
		{
				hum_req = 0;
		}
		else
		{
				hum_req = p_algorithm(current_hum,set_hum,g_sys.config.algorithm.hum_deadband,g_sys.config.algorithm.hum_precision);		
		}
		return hum_req;		
}

static uint8_t find_max_serial(int16_t array[])
{
    extern team_local_st team_local_inst;
    uint8_t ret, i, total_cnt;
    int16_t temp;
    ret   = 0;
    temp  = -32768;
  
    total_cnt = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
  
    for(i = 0; i < total_cnt; i++)
    {
        if(array[i] > temp)
        {
            temp = array[i];
            ret = i;
        }
    }
    return ret;
}

static uint8_t find_min_serial(int16_t array[])
{
    extern team_local_st team_local_inst;
    uint8_t ret, i, total_cnt;
    int16_t temp;
    ret   = 0;
    temp  = 32767;
  
    total_cnt = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
  
    for(i = 0; i < total_cnt; i++)
    {
        if(array[i] < temp)
        {
            temp = array[i];
            ret = i;
        }
    }
    return ret;
}

static void choose_sort(int16_t array[], int16_t direct, uint8_t addr_array[])  
{  
    extern team_local_st team_local_inst;
    uint8_t i, j, total_cnt;
    j = 0;
  
    total_cnt = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
  
    for(i = 0; i < total_cnt; i++)
    {
        if(team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0)
        {
            if(direct > 0)    //总需求大于0，按照降序排列
            {
                addr_array[j] = find_max_serial(array);
                array[addr_array[j]]      = -32768;
            }
            else
            {                 //总需求小于0，按照升序排列
                addr_array[j] = find_min_serial(array);
                array[addr_array[j]]      = 32767;
            }
            j++;
        }
    }
}

int16_t team_config_temp_req_calc(void)
{
    extern team_local_st  team_local_inst;
    extern sys_reg_st		  g_sys;
    uint8_t temp_ctrl_mode, target_mode;
    uint16_t t_deadband, t_precision;
    int16_t set_temp, current_temp;
    int16_t temp_req;
    //temp_REQ
    temp_ctrl_mode  = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TMODE_BPOS) & 0x0001;
    target_mode     = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TARGET_BPOS) & 0x0001;
    t_deadband      = team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND];
		t_precision     = team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION];
    set_temp        = (int16_t)team_local_inst.team_config[TEAM_CONF_TEMP_SETVAL];
  
    if(target_mode == TARGET_MODE_RETURN)
		{
				current_temp = g_sys.status.sys_tem_hum.return_air_temp;	
		}
		else
		{
				current_temp = g_sys.status.sys_tem_hum.supply_air_temp;	
		}
		if(current_temp >= 0x7fff)
		{
				return 0;
		}
    
    if((current_temp > TEMP_MIN) && (current_temp < TEMP_MAX))
    {
        switch(temp_ctrl_mode)
        {
            case(P_ALOGORITHM):
              temp_req = p_algorithm(current_temp, set_temp, t_deadband, t_precision);
              break;
            case(PID_ALOGORITHM):
              temp_req = p_algorithm(current_temp, set_temp, t_deadband, t_precision);
              temp_req = pid_temp_algorithm(temp_req, &pid_reg_inst.p_saved, &pid_reg_inst.i_saved, &pid_reg_inst.req_saved);
              break;
            default:
              temp_req = 0;
              break;
        }
    }
    else
    {
        temp_req = 0;
    }
    return temp_req;
}

int16_t team_config_hum_req_calc(void)
{
    extern team_local_st  team_local_inst;
    extern sys_reg_st		  g_sys;
    uint8_t hum_ctrl_mode, target_mode;
    uint16_t h_deadband, h_precision;
    int16_t set_hum, current_hum;
    int16_t hum_req;
    //temp_REQ
    hum_ctrl_mode   = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_HMODE_BPOS) & 0x0001;
    target_mode     = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TARGET_BPOS) & 0x0001;
    h_deadband      = team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND];
		h_precision     = team_local_inst.team_config[TEAM_CONF_HUM_PRECISION];
    set_hum         = (int16_t)team_local_inst.team_config[TEAM_CONF_HUM_SETVAL];
  
    if(target_mode == TARGET_MODE_RETURN)
		{
				current_hum = g_sys.status.sys_tem_hum.return_air_hum;
		}
		else
		{
				current_hum = g_sys.status.sys_tem_hum.supply_air_hum;	
		}
    if(current_hum >= 0x7fff)
		{
				return 0;
		}
    if(hum_ctrl_mode == HUM_ABSOLUTE)
		{
				current_hum = abs_hum_calc(current_hum);
		}
    if((current_hum > HUM_MIN) && (current_hum < HUM_MAX))
    {
        hum_req = p_algorithm(current_hum, set_hum, h_deadband, h_precision);
    }
    else
    {
        hum_req = 0;
    }
    return hum_req;
}


void team_table_req_update(void)
{
    uint8_t i;
    uint8_t temp_mode, hum_mode, total_cnt;
//    int16_t temp_req, hum_req;
    int16_t current_temp, current_hum;
    int16_t set_temp, set_hum;
    uint16_t t_deadband, t_precision;
    uint16_t h_deadband, h_precision;
    extern team_local_st team_local_inst;
    int16_t temp_sort[TEAM_MAX_SLAVE_NUM];
  
    total_cnt = (team_local_inst.team_config[TEAM_CONF_CNT] & 0x00ff);
  
    //temp_REQ
    temp_mode       = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_TMODE_BPOS) & 0x0001;
    t_deadband      = team_local_inst.team_config[TEAM_CONF_TEMP_DEADBAND];
		t_precision     = team_local_inst.team_config[TEAM_CONF_TEMP_PRECISION];
    set_temp        = (int16_t)team_local_inst.team_config[TEAM_CONF_TEMP_SETVAL];
    for(i = 0; i < total_cnt; i++)
    {
        current_temp = (int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP];
        if((team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP] > TEMP_MIN) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP] < TEMP_MAX))
        {
            switch(temp_mode)
            {
                case(P_ALOGORITHM):
                {						
                    team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ] = p_algorithm(current_temp, set_temp, t_deadband, t_precision);
                    break;
                }
                case(PID_ALOGORITHM):
                {						
                    team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ] = p_algorithm(current_temp, set_temp, t_deadband, t_precision);
                    team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ] = pid_temp_algorithm(team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ], &pid_reg_inst.p_saved, &pid_reg_inst.i_saved, &pid_reg_inst.req_saved);
                    break;
                }
                default:
                {
                    team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ] = 0;
                    break;
                }
            }
        }
        else
        {
            team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ] = 0;
        }
        temp_sort[i]= (int16_t)team_local_inst.team_table[i][TEAM_TAB_TEMP_REQ];
    }
    //temp Sort
    choose_sort(temp_sort, team_local_inst.team_param[TEAM_PARAM_TOTAL_TEMP_REQ], team_local_inst.team_sort_temp);
    
    //hum_REQ
    hum_mode        = (team_local_inst.team_config[TEAM_CONF_MODE] >> TEAM_CONF_HMODE_BPOS) & 0x0001;
    h_deadband      = team_local_inst.team_config[TEAM_CONF_HUM_DEADBAND];
		h_precision     = team_local_inst.team_config[TEAM_CONF_HUM_PRECISION];
    set_hum         = (int16_t)team_local_inst.team_config[TEAM_CONF_HUM_SETVAL];
    for(i = 0; i < total_cnt; i++)
    {
        current_hum = (int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM];
        if(hum_mode == HUM_ABSOLUTE)
        {
            current_hum = abs_hum_calc(current_hum);
        }
        if((team_local_inst.team_table[i][TEAM_TAB_TIMEOUT] > 0) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM] > HUM_MIN) &&
          ((int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM] < HUM_MAX))
        {
            team_local_inst.team_table[i][TEAM_TAB_HUM_REQ] = p_algorithm(current_hum, set_hum, h_deadband, h_precision);
        }
        else
        {
            team_local_inst.team_table[i][TEAM_TAB_HUM_REQ] = 0;
        }
        temp_sort[i] = (int16_t)team_local_inst.team_table[i][TEAM_TAB_HUM_REQ];
    }
    
    //hum Sort
    choose_sort(temp_sort, team_local_inst.team_param[TEAM_PARAM_TOTAL_HUM_REQ], team_local_inst.team_sort_hum);
}

void req_update(void)
{
		extern local_reg_st l_sys;
		l_sys.require[LOCAL_REQ][T_REQ] = local_temp_req_calc();
		l_sys.require[LOCAL_REQ][H_REQ] = local_hum_req_calc();
//		l_sys.require[TEAM_REQ][T_REQ] = team_temp_req_calc();
//		l_sys.require[TEAM_REQ][H_REQ] = team_hum_req_calc();		
}

