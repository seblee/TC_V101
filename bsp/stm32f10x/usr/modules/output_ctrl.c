#include <rtthread.h>
#include "sys_conf.h"
#include "local_status.h"
//#include "authentication.h"
#include "dio_bsp.h"
#include "pwm_bsp.h"

//static void manual_ao_op(uint8_t component_bpos, int16_t value)
//{
//		extern local_reg_st l_sys;
//
//		l_sys.ao_list[component_bpos][BITMAP_MANUAL] = value;
//}
//
//static void final_ao_op(uint8_t component_bpos, int16_t value)
//{
//		extern local_reg_st l_sys;
//
//		l_sys.ao_list[component_bpos][BITMAP_FINAL] = value;
//}


/**
  * @brief 	output control module dout and system status update 
	* @param  none
	* @retval none
  */
static void oc_do_update(uint16_t new_bitmap)
{		
		extern sys_reg_st		g_sys;
		extern local_reg_st	l_sys;
		uint16_t xor_bitmap,old_bitmap;
		uint16_t i;
		old_bitmap = g_sys.status.dout_bitmap;
		xor_bitmap = new_bitmap ^ old_bitmap;		
		if(xor_bitmap != 0)																					//if output bitmap changed
		{
				for(i=0;i<16;i++)
				{
						if(((xor_bitmap>>i)&0x0001) != 0)										//do status change
						{
								if(((new_bitmap>>i)&0x0001) != 0)
								{
										dio_set_do(i+1,Bit_SET);
								}
								else
								{
										dio_set_do(i+1,Bit_RESET);
								}
						}
						else																											//do status no change, continue for loop
						{
								continue;
						}
				}						
				g_sys.status.dout_bitmap = new_bitmap;		//update system dout bitmap
		}
		else																															//output bitmap unchange
		{
				;
		}
//		authen_expire_cd();
}

/**
  * @brief 	system output arbitration
	* @param  none
	* @retval final output bitmap
  */

static uint16_t oc_arbitration(void)
{		
		extern sys_reg_st			g_sys;
		extern local_reg_st 	l_sys;
		uint16_t cat_bitmap;
		uint16_t sys_bitmap;
		uint16_t alarm_bitmap,bitmap_mask,bitmap_mask_reg,bitmap_mask_reset;
		uint16_t target_bitmap;
		uint16_t final_bitmap;
		
		bitmap_mask_reset = 0;
		alarm_bitmap = l_sys.bitmap[BITMAP_ALARM];
		bitmap_mask = l_sys.bitmap[BITMAP_MASK];
	
		cat_bitmap = l_sys.bitmap[BITMAP_REQ];
		
		if((g_sys.config.general.diagnose_mode_en == 0)&&(g_sys.config.general.testing_mode_en == 0))//if diagnose enable, output manual, else out put concatenated bitmap
		{
				target_bitmap = cat_bitmap;
				l_sys.bitmap[BITMAP_MANUAL] = l_sys.bitmap[BITMAP_FINAL];
		}
		else
		{
				target_bitmap = l_sys.bitmap[BITMAP_MANUAL];
		}                   
		
		bitmap_mask_reg = (g_sys.config.general.alarm_bypass_en == 0)? bitmap_mask : bitmap_mask_reset; //bitmap mask selection, if alarm_bypass_en set, output reset bitmap

		sys_bitmap = (target_bitmap & ~bitmap_mask_reg) | (alarm_bitmap & bitmap_mask_reg);		//sys_out_bitmap output		
		
		final_bitmap = (g_sys.config.general.testing_mode_en == 0)? sys_bitmap : l_sys.bitmap[BITMAP_MANUAL];	//final bitmap selection, if test mode enable, output manual, otherwise sys_bitmap
	
		l_sys.bitmap[BITMAP_FINAL]= final_bitmap & g_sys.config.dev_mask.dout;
		
		return l_sys.bitmap[BITMAP_FINAL];
}


//static void oc_ao_arbitration(void)
//{
//		extern sys_reg_st			g_sys;
//		extern local_reg_st 	l_sys;
//		uint16_t i;
//		
//		if((g_sys.config.general.diagnose_mode_en == 0)&&(g_sys.config.general.testing_mode_en == 0))//if diagnose enable, output manual, else out put concatenated bitmap
//		{
//				for(i=0;i<AO_MAX_CNT;i++)
//				{
//						manual_ao_op(i,g_sys.status.aout[i]);	
//						final_ao_op(i,l_sys.ao_list[i][BITMAP_REQ]);
//				}
//		}
//		else
//		{
//				for(i=0;i<AO_MAX_CNT;i++)
//				{
//						final_ao_op(i,l_sys.ao_list[i][BITMAP_MANUAL]);
//				}
//		}
//}


//static void oc_ao_update(void)
//{
//		extern sys_reg_st			g_sys;
//		extern local_reg_st 	l_sys;
//		uint16_t i;
//		
//		for(i=0;i<AO_MAX_CNT;i++)
//		{
//				if(g_sys.config.dev_mask.aout&(0x0001<<i))
//				{
//					if(g_sys.status.aout[i] != l_sys.ao_list[i][BITMAP_FINAL])
//					{
//							g_sys.status.aout[i] = l_sys.ao_list[i][BITMAP_FINAL];
//							pwm_set_ao(i+1,g_sys.status.aout[i]);
//					}
//				}
//				else
//				{
//					if(g_sys.status.aout[i] != 0)
//					{
//							g_sys.status.aout[i] = 0;
//							pwm_set_ao(i+1,0);
//					}
//				}
//		}
//}


/**
  * @brief 	update system output reffering to local bitmaps
	* @param  none
	* @retval none
  */
void oc_update(void)
{
		uint16_t final_bitmap;
		final_bitmap = oc_arbitration();
		oc_do_update(final_bitmap);
//		oc_ao_arbitration();
//		oc_ao_update();
}
