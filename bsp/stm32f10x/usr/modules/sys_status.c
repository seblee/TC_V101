#include <rtthread.h>
#include "team.h"
#include "sys_conf.h"
#include "local_status.h"
#include "dio_bsp.h"

void sys_set_remap_status(uint8_t reg_no, uint8_t sbit_pos, uint8_t bit_action)
{
		extern sys_reg_st			g_sys;
		if(bit_action == 1)
		{
				g_sys.status.status_remap[reg_no] |= (0x0001<<sbit_pos);		
		}
		else
		{
				g_sys.status.status_remap[reg_no] &= ~(0x0001<<sbit_pos);
		}
}

uint16_t sys_get_remap_status(uint8_t reg_no,uint8_t rbit_pos)
{
		extern sys_reg_st			g_sys;
		return ((g_sys.status.status_remap[reg_no] >> rbit_pos) & 0x0001);		
}

//uint8_t sys_get_di_sts(uint8_t din_channel) 
//{
//		uint8_t byte_offset,bit_offset;
//		extern sys_reg_st			g_sys;
//	
//		byte_offset =  din_channel >> 4;
//		bit_offset = din_channel & 0x0f;
//		if((g_sys.status.din_bitmap[byte_offset]>>bit_offset) & 0X0001)
//		{
//				return 1;
//		}
//		else
//		{
//				return 0;
//		}	
//		
//		
//}

//void sys_option_di_sts(uint8_t din_channel,uint8_t option) 
//{
//		uint8_t byte_offset,bit_offset;
//		extern sys_reg_st			g_sys;
//	
//		byte_offset =  din_channel >> 4;
//		bit_offset = din_channel & 0x0f;
//		
//		if(option)
//		{
//				g_sys.status.din_bitmap[byte_offset] |=(0x0001 << bit_offset);
//		}
//		else
//		{
//				g_sys.status.din_bitmap[byte_offset] &=(~(0x0001 << bit_offset));
//		}
//}
uint8_t sys_get_do_sts(uint8_t dout_channel) 
{
		extern sys_reg_st			g_sys;		
		if(g_sys.status.dout_bitmap & (0x0001<<dout_channel))
		{
				return 1;
		}
		else
		{
				return 0;
		}	
}


//uint16_t sys_get_pwr_signal(void)
//{
//		extern sys_reg_st			g_sys;
//    uint8_t ret;
//	
//		if((sys_get_di_sts(DI_REMOTE_BPOS) != 0)||
//				(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_LOCK )||
//				(g_sys.status.sys_work_mode.work_mode == WORK_MODE_FSM_MANAGE)||
//				((g_sys.config.general.power_mode == 0)&&(g_sys.config.team.team_en == 0))||
//				((g_sys.config.team.team_en == 1)&&(team_get_pwr_sts() == 0)))
//		{
//        ret = 0;
//		}
//		else
//		{
//        ret = 1;
//		}
////    rt_kprintf("pwr_signal is %x \n", ret);
//    return ret;
//}

//void sys_running_mode_update(void)
//{
//		extern sys_reg_st			g_sys;
//		extern local_reg_st l_sys;
//		//FAN STATUS UPDATE
//		
//		if(g_sys.config.team.team_en == 1)
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,TEAM_STANDALONE_STS_BPOS, 1);
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,TEAM_STANDALONE_STS_BPOS, 0);
//		}
//
//
//		if(sys_get_pwr_signal() ==1)
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,PWR_STS_BPOS, 1);
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,PWR_STS_BPOS, 0);
//		}		
//		
//		
//		g_sys.status.general.running_mode = (g_sys.config.general.testing_mode_en<<2)|(g_sys.config.general.diagnose_mode_en<<1)|(g_sys.config.general.alarm_bypass_en<<0);
//
//		//set fan status
//		if(sys_get_do_sts(DO_FAN_BPOS) == 1)
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS,1);
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,FAN_STS_BPOS,0);
//		}	
//		
//		//set cooling status
//		if(((sys_get_do_sts(DO_COMP1_BPOS) == 1)||(sys_get_do_sts(DO_COMP2_BPOS) == 1)||(sys_get_do_sts(DO_WATER_VALVE_DUMMY_BPOS) == 1))&&(l_sys.require[TARGET_REQ][T_REQ] > 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO,DEMHUM_STS_BPOS) == 0))
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,COOLING_STS_BPOS,1);				
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,COOLING_STS_BPOS,0);
//		}
//				
//		//set heating status
//		if((sys_get_do_sts(DO_RH1_BPOS) == 1)||(sys_get_do_sts(DO_RH2_BPOS) == 1)/*||(sys_get_do_sts(DO_RH3_BPOS) == 1)*/)
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,HEATING_STS_BPOS,1);
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,HEATING_STS_BPOS,0);
//		}		
//
//		//set hum status
//		if(sys_get_do_sts(DO_HUM_BPOS) == 1)
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,HUMING_STS_BPOS,1);
//		}
//		else
//		{
//				sys_set_remap_status(WORK_MODE_STS_REG_NO,HUMING_STS_BPOS,0);
//		}				
//}

//uint16_t sys_get_pwr_sts(void)
//{
//		extern sys_reg_st			g_sys;
//		if((g_sys.status.status_remap[WORK_MODE_STS_REG_NO]>>PWR_STS_BPOS) & 0X0001)
//		{
//				return 1;
//		}
//		else
//		{
//				return 0;
//		}
//}
//uint8_t sys_get_mbm_online(uint8_t mbm_dev)
//{
//		extern sys_reg_st			g_sys;
//		if((g_sys.status.status_remap[MBM_COM_STS_REG_NO]) & (0X01<<mbm_dev))
//		{
//				return(1);
//		}
//		else
//		{
//				return(0);
//		}
//}
//
//uint16_t devinfo_get_compressor_cnt(void)
//{
//		extern sys_reg_st			g_sys;
//		uint16_t compressor_bit_map;
//		uint16_t compressor_count;
//	
//		compressor_bit_map = (((g_sys.config.dev_mask.dout >>DO_COMP2_BPOS)&0x0001)<<1);
//		compressor_bit_map |= ((g_sys.config.dev_mask.dout >>DO_COMP1_BPOS)&0x0001);		
//	
//		switch(compressor_bit_map)
//		{
//				case (0):
//				{
//						compressor_count = 0;
//						break;
//				}
//				case (1):
//				{
//						compressor_count = 1;
//						break;
//				}
//				case (3):
//				{
//						compressor_count = 2;
//						break;
//				}
//				default:
//				{
//						compressor_count = 0;
//						break;
//				}				
//		}
//		return compressor_count;
//}
//
//
//uint16_t devinfo_get_heater_level(void)
//{
//		extern sys_reg_st			g_sys;
//		uint16_t herter_bit_map;
//		uint16_t heater_level;		
//	
//		herter_bit_map = (((g_sys.config.dev_mask.dout >>DO_RH2_BPOS)&0x0001)<<1);
//		herter_bit_map |= ((g_sys.config.dev_mask.dout >>DO_RH1_BPOS)&0x0001);
//	
//		switch(herter_bit_map)
//		{
//				case (0):
//				{
//						heater_level = 0;
//						break;
//				}
//				case (1):
//				{
//						heater_level = 1;
//						break;
//				}
//				case (3):
//				{
//						heater_level = 3;
//						break;
//				}
//				default:
//				{
//						heater_level = 0;
//						break;
//				}				
//		}
//		return heater_level;
//}

