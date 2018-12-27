#include <rtthread.h>
#include <components.h>
#include "daq.h"
#include "sys_conf.h"
#include "sys_status.h"
#include "calc.h"

#define NTC_TEMP_SCALE   191
#define NTC_TEMP_OFFSET  39		
#define NTC_TEMP_DT 15
extern  sys_reg_st g_sys;

const uint16_t ntc_lookup_tab[NTC_TEMP_SCALE] = 
{
	193,204,215,227,239,252,265,279,294,309,324,340,357,375,393,411,431,451,471,
	492,514,537,560,584,609,635,661,687,715,743,772,801,831,862,893,925,957,991,
  1024,1058,1093,1128,1164,1200,1236,1273,1310,1348,1386,1424,1463,1501,1540,
  1579,1618,1657,1697,1736,1775,1815,1854,1893,1932,1971,2010,2048,2087,2125,
  2163,2200,2238,2274,2311,2347,2383,2418,2453,2488,2522,2556,2589,2622,2654,
  2685,2717,2747,2777,2807,2836,2865,2893,2921,2948,2974,3000,3026,3051,3075,
  3099,3123,3146,3168,3190,3212,3233,3253,3273,3293,3312,3331,3349,3367,3384,
  3401,3418,3434,3450,3465,3481,3495,3510,3524,3537,3551,3564,3576,3588,3600,
  3612,3624,3635,3646,3656,3667,3677,3686,3696,3705,3714,3723,3732,3740,3748,
  3756,3764,3772,3779,3786,3793,3800,3807,3813,3820,3826,3832,3838,3844,3849,
  3855,3860,3865,3870,3875,3880,3884,3889,3893,3898,3902,3906,3910,3914,3918,
  3922,3925,3929,3932,3936,3939,3942,3945,3949,3952,3954,3957,3960,3963,3966,
  3968,3971,3973
};





static int16_t calc_ntc(uint16_t adc_value,int16_t adjust)
{	
		int16_t ntc_temp;
		int16_t index;
		uint16_t offset;
		adc_value= 4096 - adc_value;
		index = bin_search((uint16_t *)ntc_lookup_tab, NTC_TEMP_SCALE, adc_value);  		
		if(index < 0)
		{
				return 0x7fff;				
		}
		else
		{
				offset = (adc_value-ntc_lookup_tab[index-1])*10 / (ntc_lookup_tab[index]-ntc_lookup_tab[index-1]);
				ntc_temp = ( index- NTC_TEMP_OFFSET)*10 + offset + adjust - NTC_TEMP_DT;
				return ntc_temp;
		}	
			
}

static int16_t get_average_temp(uint8_t type)
{
	 uint8_t index=0,i;
	 int16_t sum=0;	 

	 if(type == TARGET_MODE_RETURN)
	 {
	 	 //硬件
		 //配置信息 温度传感器 前四个为回风，NTC 前面两个 为回风温度
		 //	g_sys.config.
		 //报警信息
		//alrmbit= get_alarm_status_bitmap(ACL_TNH_MALFUNC>>4,ACL_TNH_MALFUNC&0x0f);
		//TEM_SENSOR
			 if(((g_sys.config.dev_mask.mb_comp) & (0X01<<RETURN_TEM_HUM_SENSOR_BPOS))&&
				(sys_get_mbm_online(RETURN_TEM_HUM_SENSOR_BPOS) == 1))//配置位 online
			 {
					if(sys_get_remap_status(SENSOR_STS_REG_NO,RETURN_TEM_HUM_SENSOR_BPOS) == 0 )//报警位
					{
						sum +=(int16_t) (g_sys.status.mbm.tnh[RETURN_TEM_HUM_SENSOR_BPOS].temp);
						index++;
					}
			 }
		 	//NTC 
			if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC1_BPOS))
				 {
					 if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC1_FAULT_BPOS) == 0)//报警位
						{
								sum +=(int16_t)( g_sys.status.ain[DEV_AIN_RETUREN_NTC1_BPOS]);
								index++;
						}
				 }
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC2_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC2_FAULT_BPOS) == 0)//报警位
						{
								sum += (int16_t)(g_sys.status.ain[DEV_AIN_RETUREN_NTC2_BPOS]);
								index++;
						}
				 }
			}
	 }
	 else if(type == TARGET_MODE_SUPPLY)
	 {
			//TEM_SENSOR
			 if(((g_sys.config.dev_mask.mb_comp) & (0X01<<SUPPLY_TEM_HUM_SENSOR_BPOS))&&
				(sys_get_mbm_online(SUPPLY_TEM_HUM_SENSOR_BPOS) == 1))//配置位 online
			 {
					if(sys_get_remap_status(SENSOR_STS_REG_NO,SUPPLY_TEM_HUM_SENSOR_BPOS) == 0)//报警位
					{
						sum +=(int16_t)( g_sys.status.mbm.tnh[SUPPLY_TEM_HUM_SENSOR_BPOS].temp);
						index++;
					}
			 }
			 //NTC 
			 if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			 {
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC1_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC1_FAULT_BPOS) == 0)//报警位
						{
							sum += (int16_t)(g_sys.status.ain[DEV_AIN_SUPPLY_NTC1_BPOS]);
							index++;
						}
				 }
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC2_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC2_FAULT_BPOS) == 0)//报警位
						{
							sum += (int16_t)(g_sys.status.ain[DEV_AIN_SUPPLY_NTC2_BPOS]);
							index++;
						}
				 }
			 }
		
	 }
	 else//REMOTE TARGET
	 {
			if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
					for(i = TEM_HUM_SENSOR3_BPOS ;i <= TEM_HUM_SENSOR8_BPOS;i++ )
					{
							if(((g_sys.config.dev_mask.mb_comp) & (0X01<<i))&&
							(sys_get_mbm_online(i) == 1))//配置位 online
							{
								if(sys_get_remap_status(SENSOR_STS_REG_NO,i) == 0 )//报警位
								{
									sum +=(int16_t) (g_sys.status.mbm.tnh[i].temp);
									index++;
								}
							}
					} 
			}
	 }
	if(index!=0)
	{
	 return(sum/index);
	}
	else
	{
		return(0x7fff);
	}
}
static uint16_t get_max_temp(uint8_t type)
{
	 uint8_t index=0,i;
	 int16_t meter,max=0x8000;	 

	 if(type == TARGET_MODE_RETURN)
	 {
	 	 //硬件
		 //配置信息 温度传感器 前四个为回风，NTC 前面两个 为回风温度
		 //	g_sys.config.
		 //报警信息
		//alrmbit= get_alarm_status_bitmap(ACL_TNH_MALFUNC>>4,ACL_TNH_MALFUNC&0x0f);
		//TEM_SENSOR
		 if(((g_sys.config.dev_mask.mb_comp) & (0X01<<RETURN_TEM_HUM_SENSOR_BPOS))&&
		 	(sys_get_mbm_online(RETURN_TEM_HUM_SENSOR_BPOS) == 1))//配置位 online
		 {
				if(sys_get_remap_status(SENSOR_STS_REG_NO,RETURN_TEM_HUM_SENSOR_BPOS) == 0 )//报警位
				{
					max =(int16_t) (g_sys.status.mbm.tnh[RETURN_TEM_HUM_SENSOR_BPOS].temp);
					index++;
				}
		 }
		 
		 //NTC 
		 if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC1_BPOS))
				 {
					 if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC1_FAULT_BPOS) == 0)//报警位
						{
								meter =(int16_t)( g_sys.status.ain[DEV_AIN_RETUREN_NTC1_BPOS]);
								if(meter > max)
								{
										max =  meter;
								}
								index++;
						}
				 }
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC2_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC2_FAULT_BPOS) == 0)//报警位
						{
								meter= (int16_t)(g_sys.status.ain[DEV_AIN_RETUREN_NTC2_BPOS]);
								if(meter > max)
								{
										max =  meter;
								}
								index++;
						}
				 }
			 }
		 
	 }
	 else if(type == TARGET_MODE_SUPPLY)
	 {
	 	//TEM_SENSOR
		 if(((g_sys.config.dev_mask.mb_comp) & (0X01<<SUPPLY_TEM_HUM_SENSOR_BPOS))&&
		 	(sys_get_mbm_online(SUPPLY_TEM_HUM_SENSOR_BPOS) == 1))//配置位 online
		 {
				if(sys_get_remap_status(SENSOR_STS_REG_NO,SUPPLY_TEM_HUM_SENSOR_BPOS) == 0)//报警位
				{
					max =(int16_t)( g_sys.status.mbm.tnh[SUPPLY_TEM_HUM_SENSOR_BPOS].temp);
					index++;
				}
		 }
		 //NTC 
		  if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC1_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC1_FAULT_BPOS) == 0)//报警位
						{
							meter = (int16_t)(g_sys.status.ain[DEV_AIN_SUPPLY_NTC1_BPOS]);
							if(meter > max)
							{
										max =  meter;
							}
							index++;
						}
				 }
				 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC2_BPOS))
				 {
						if(sys_get_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC2_FAULT_BPOS) == 0)//报警位
						{
						   meter = (int16_t)(g_sys.status.ain[DEV_AIN_SUPPLY_NTC2_BPOS]);
							 if(meter > max)
							 {
									max =  meter;
							 }
							 index++;
						}
				 }
		  }
		
	 }
	 else //remote target
	 {
			if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
					for(i = TEM_HUM_SENSOR3_BPOS ;i <= TEM_HUM_SENSOR8_BPOS;i++ )
					{
							if(((g_sys.config.dev_mask.mb_comp) & (0X01<<i))&&
							(sys_get_mbm_online(i) == 1))//配置位 online
							{
								if(sys_get_remap_status(SENSOR_STS_REG_NO,i) == 0 )//报警位
								{
									meter =(int16_t) (g_sys.status.mbm.tnh[i].temp);
									if(meter > max )
									{
										  max  = meter;
									}
									index++;
								}
							}
					} 
			}
	 }
	if(index!=0)
	{
	 return(max);
	}
	else
	{
		return(0x7fff);
	}
}
int16_t get_current_temp(uint8_t type)
{
		int16_t temp;
	 if(g_sys.config.algorithm.temp_calc_mode == MAX_TEMP_MODE)
	 {
		 temp=get_max_temp(type);
	 }
	 else
	 {
		 temp=get_average_temp(type);
	 }

	 return(temp);
}

uint16_t get_current_hum(uint8_t type)
{
	uint16_t sum=0;
	uint8_t index=0,i;
	if(type == TARGET_MODE_RETURN)
	{
		  if(((g_sys.config.dev_mask.mb_comp) & (0X01<<RETURN_TEM_HUM_SENSOR_BPOS))&&
		 	(sys_get_mbm_online(RETURN_TEM_HUM_SENSOR_BPOS) == 1))//配置位 online
		 {
					if(sys_get_remap_status(SENSOR_STS_REG_NO,RETURN_TEM_HUM_SENSOR_BPOS) == 0)//报警位
					{
							sum += g_sys.status.mbm.tnh[RETURN_TEM_HUM_SENSOR_BPOS].hum;
							index++;
					}
		 } 
	}
	else if(type == TARGET_MODE_SUPPLY)
	{
		  if(((g_sys.config.dev_mask.mb_comp) & (0X01<<SUPPLY_TEM_HUM_SENSOR_BPOS))&&
		 	(sys_get_mbm_online(SUPPLY_TEM_HUM_SENSOR_BPOS) == 1))//配置位 onlin
		 {
					if(sys_get_remap_status(SENSOR_STS_REG_NO,SUPPLY_TEM_HUM_SENSOR_BPOS) == 0)//报警位
					{
							sum += g_sys.status.mbm.tnh[SUPPLY_TEM_HUM_SENSOR_BPOS].hum;
							index++;
					}
		 }  
	}
	else //remote target
	{
		  if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)||((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER))) 
			{
					for(i = TEM_HUM_SENSOR3_BPOS ;i <= TEM_HUM_SENSOR8_BPOS;i++ )
					{
							if(((g_sys.config.dev_mask.mb_comp) & (0X01<<i))&&
							(sys_get_mbm_online(i) == 1))//配置位 online
							{
									if(sys_get_remap_status(SENSOR_STS_REG_NO,i) == 0 )//报警位
									{
											sum +=(int16_t) (g_sys.status.mbm.tnh[i].hum);
											index++;
									}
							}
					} 
			}
	}
	
	if(index!=0)
	{
		 return(sum/index);
	}
	else
	{
		return(0x7fff);
	}

}

#define Rs  200

static int16_t calc_10vai(uint16_t adc_value)
{
	int16_t relvaule;
	relvaule = (adc_value*132)/4096;
	return(relvaule);
}

static int16_t calc_ap_ai(uint16_t adc_value)
{
	int16_t relvaule;	
	relvaule = (adc_value*1330-100)/(16384*Rs);
	return(relvaule);
}

void ai_sts_update(sys_reg_st*	gds_sys_ptr)
{
		extern volatile uint16_t ADC1ConvertedValue[AI_MAX_CNT];
		uint16_t ain_mask_bitmap;
		uint16_t i;
		ain_mask_bitmap = gds_sys_ptr->config.dev_mask.ain;
		for(i=0;i<4;i++)
		{
				gds_sys_ptr->status.ain[i] = (((ain_mask_bitmap>>i)&0x0001) != 0)? calc_10vai(ADC1ConvertedValue[i]):0;
		}
		
		//caculate air flow difference
		if((ain_mask_bitmap&(0x0001<<AI_AIR_FLOW_DIFF)) != 0)
		{
				gds_sys_ptr->status.ain[AI_AIR_FLOW_DIFF] = calc_ap_ai(ADC1ConvertedValue[AI_AIR_FLOW_DIFF]);
		}
		
		for(i=4;i<AI_MAX_CNT;i++)
		{
				gds_sys_ptr->status.ain[i] = (((ain_mask_bitmap>>i)&0x0001) != 0)?  calc_ntc(ADC1ConvertedValue[i],gds_sys_ptr->config.general.ntc_cali[i-4]):0;
		}
}


FINSH_FUNCTION_EXPORT(calc_ntc, calc ntc);

