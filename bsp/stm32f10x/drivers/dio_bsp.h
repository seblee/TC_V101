#ifndef __DIO_H__
#define __DIO_H__
#include "stm32f10x.h"
#include "sys_conf.h"

enum
{
		DI_HI_PRESS1_BPOS	 =0,
		DI_LO_PRESS1_BPOS,
	  DI_HI_PRESS2_BPOS,	
		DI_LO_PRESS2_BPOS,					
		DI_FAN01_OD_BPOS,						
		DI_AIR_FLOW_BPOS,					
		DI_SMOKE_BPOS,							
		DI_WATER_LEAK_BPOS,				
		DI_HEATER_OD_BPOS,					
		DI_REMOTE_BPOS,						
		DI_FILTER_BPOS,						
		DI_REV_BPOS,
		DI_FAN02_OD_BPOS,
		DI_FAN03_OD_BPOS,
		DI_FAN04_OD_BPOS,					
		DI_FAN05_OD_BPOS,
		DI_HUM_DEFAULT_BPOS,
		DI_GROUP_DEFAULT_BPOS,
		DI_POWER_LOSS_BPOS,
		DI_DUMMY_BPOS
		
};
#define  DI_USR_BPOS            DI_SMOKE_BPOS
#define  DI_BLOCKING_BPOS       DI_LO_PRESS2_BPOS
//column air condition  fan di
#define  DI_FAN06_OD_BPOS       DI_HI_PRESS1_BPOS
#define  DI_FAN07_OD_BPOS       DI_LO_PRESS1_BPOS
#define  DI_FAN08_OD_BPOS       DI_HI_PRESS2_BPOS
#define  DI_COND_HI_LEVEL_BPOS  DI_REV_BPOS
#define  DI_COND_LO_LEVEL_BPOS  DI_AIR_FLOW_BPOS
//huawei  dummy
#define DI_EXT_TEMP1_BPOS   DI_DUMMY_BPOS
#define DI_EXT_TEMP2_BPOS   DI_DUMMY_BPOS
#define DI_OSD_OUT_FAN_BPOS DI_DUMMY_BPOS
#define MBM_OSC_DI_OUT_FAN_BPOS DI_DUMMY_BPOS
#define DI_BK_POWER_BPOS   DI_DUMMY_BPOS
#define DI_OSD_INVERTER_BPOS DI_DUMMY_BPOS





#define		DI_WATER_FLOW_BPOS				DI_LO_PRESS2_BPOS		//chilled water cool type
//#define		DI_HI_PRESS1_BPOS					0
//#define		DI_LO_PRESS1_BPOS					1
//#define		DI_HI_PRESS2_BPOS					2
//#define		DI_LO_PRESS2_BPOS					3
//#define		DI_FAN_OD_BPOS						4
//#define		DI_AIR_FLOW_BPOS					5
//#define		DI_SMOKE_BPOS							6
//#define		DI_WATER_LEAK_BPOS				7
//#define		DI_HEATER_OD_BPOS					8
//#define		DI_REMOTE_BPOS						9
//#define		DI_FILTER_BPOS						10
//#define		DI_USR_BPOS								11
//#define		DI_COND_FAN_ALARM1_BPOS		12
//#define		DI_COND_FAN_ALARM2_BPOS		13
//#define		DI_EXT_TEMP1_BPOS					14
//#define		DI_EXT_TEMP2_BPOS					15



void drv_dio_init(void);
void di_sts_update(sys_reg_st*	gds_sys_ptr);
void dio_set_do(uint16_t channel_id, BitAction data);

#endif //__DIO_H__
