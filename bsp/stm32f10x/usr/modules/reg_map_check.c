#include "sys_def.h" 
#include "reg_map_check.h"
extern sys_reg_st					g_sys; 

//compressor0_uper_speed check
uint8_t comp_uper_spd_chk(uint16_t pram)
{
	if(pram > g_sys.config.compressor.speed_lower_lim)
	{
			return(1);
	}
	return(0);
}
//compressor0 lower speed check
uint8_t comp_low_spd_chk(uint16_t pram) 
{
	if(pram < g_sys.config.compressor.speed_upper_lim)
	{
			return(1);
	}
	return(0);
}

uint8_t water_valve_set_chk(uint16_t param)
{
    if((param <= g_sys.config.water_valve.max_opening) &&
       (param >= g_sys.config.water_valve.min_opening))
    {
        return (1);
    }
    return (0);
}

uint8_t water_valve_min_chk(uint16_t param)
{
    if(param < g_sys.config.water_valve.max_opening)
    {
        return(1);
    }
    return (0);
}

uint8_t water_valve_max_chk(uint16_t param)
{
    if(param > g_sys.config.water_valve.min_opening)
    {
        return (1);
    }
    return (0);
}

uint8_t fan_set_spd_chk(uint16_t pram)
{
	if((pram <= g_sys.config.fan.max_speed)&&(pram >= g_sys.config.fan.min_speed))
	{
			return(1);
	}
	return(0);
}
uint8_t fan_low_spd_chk(uint16_t pram)
{
	if((pram <= g_sys.config.fan.max_speed)&&(pram <= g_sys.config.fan.set_speed))
	{
			return(1);
	}
	return(0);
}
uint8_t fan_uper_spd_chk(uint16_t pram)
{
	if((pram >= g_sys.config.fan.min_speed)&&(pram >= g_sys.config.fan.set_speed))
	{
			return(1);
	}
	return(0);
}
uint8_t fan_dehum_min_spd_chk(uint16_t pram)
{
	if((pram <= g_sys.config.fan.max_speed)&&(pram >= g_sys.config.fan.min_speed)&&(pram <= g_sys.config.fan.set_speed))
	{
			return(1);
	}
	return(0);
}

uint8_t team_total_num_chk(uint16_t pram)
{
	if(pram > g_sys.config.team.backup_num)
	{
			return(1);
	}
	return(0);
}
uint8_t team_back_num_chk(uint16_t pram)
{
	if(pram <= (g_sys.config.team.total_num>>1))
	{
			return(1);
	}
	return(0);
}

uint8_t return_temp_hiacl_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param)
	{
			return(1);
	}
	return(0);
}

uint8_t return_temp_lowacl_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t return_hum_hiacl_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t return_hum_lowacl_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param)
	{
			return(1);
	}
	return(0);
}

uint8_t supply_temp_hiacl_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_LO_TEMP_SUPPLY].alarm_param)
	{
			return(1);
	}
	return(0);
}

uint8_t supply_temp_lowacl_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_HI_TEMP_SUPPLY].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t supply_hum_hiacl_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_LO_HUM_SUPPLY].alarm_param)
	{
			return(1);
	}
	return(0);
}
 uint8_t supply_hum_lowacl_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_HI_HUM_SUPPLY].alarm_param)
	{
			return(1);
	}
	return(0);
}

uint8_t water_elec_hi_chk(uint16_t pram)
{
		if(pram > g_sys.config.alarm[ACL_WATER_ELEC_LO].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t water_elec_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_WATER_ELEC_HI].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t power_frq_hi_chk(uint16_t pram)
{
		if(pram > g_sys.config.alarm[ACL_POWER_LO_FD].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t power_frq_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_POWER_HI_FD].alarm_param)
	{
			return(1);
	}
	return(0);
}

uint8_t power_a_hi_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_POWER_A_LOW].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t power_a_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_POWER_A_HIGH].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t power_b_hi_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_POWER_B_LOW].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t power_b_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_POWER_B_HIGH].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t power_c_hi_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_POWER_C_LOW].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t power_c_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_POWER_C_HIGH].alarm_param)
	{
			return(1);
	}
	return(0);
}
//uint8_t cool0_temp_hi_chk(uint16_t pram)
//{
//	if(pram > g_sys.config.alarm[ACL_COIL_BLOCKING].alarm_param)
//		{
//				return(1);
//		}
//		return(0);
//}
uint8_t cool0_temp_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_COIL_HI_TEM1].alarm_param)
	{
			return(1);
	}
	return(0);
}
uint8_t cool1_temp_hi_chk(uint16_t pram)
{
	if(pram > g_sys.config.alarm[ACL_FAN_OT6].alarm_param)
		{
				return(1);
		}
		return(0);
}
uint8_t cool1_temp_lo_chk(uint16_t pram)
{
	if(pram < g_sys.config.alarm[ACL_FAN_OVERLOAD7].alarm_param)
	{
			return(1);
	}
	return(0);
}


