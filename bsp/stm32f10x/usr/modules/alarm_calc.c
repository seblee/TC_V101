#include <rtthread.h>
#include "sys_conf.h"
#include "kits/fifo.h"
#include "alarms.h"
#include "local_status.h"
#include "global_var.h"
#include "req_execution.h"

#include "event_record.h"
#include "rtc_bsp.h"
#include "daq.h"
#include "dio_bsp.h"
#include "sys_status.h"
#include "team.h"
#define ACL_ENMODE_ENABLE	0x0000
#define ACL_ENMODE_SUPPRESS	0x0001//阻塞
#define ACL_ENMODE_DISABLE	0x0002



#define ACL_ENMODE_AUTO_RESET_ALARM     0X0004
#define ACL_ENMODE_HAND_RESET_ALARM     0X0000

#define MBM_DEV_STS_DEFAULT		0x8000

#define ACL_TEM_MAX 1020 //0.1℃
#define ACL_TEM_MIN -280

#define ACL_HUM_MAX 1000 // 0.1%
#define ACL_HUM_MIN  0

#define IO_CLOSE 1
#define IO_OPEN 0

#define OVER_TIME_ACCURACY     	86400          //DATE



#define POWER_PARAM		0xFF80
#define POWER_DOT 0x7f
#define POWER_DOT_BIT 7


typedef struct alarm_acl_td
{
	uint16_t				  id;
	uint16_t 					state;
	uint16_t          alram_value;
	uint16_t					timeout;
	uint16_t 					enable_mask;
	uint16_t 					reset_mask;
	uint8_t           alarm_level;
	uint16_t          dev_type;
	uint16_t (*alarm_proc)(struct alarm_acl_td* str_ptr);	
	
}alarm_acl_status_st;


typedef enum
{
	SML_MIN_TYPE=0,
	THR_MAX_TYPE,
	OUT_MIN_MAX_TYPE,
	IN_MIN_MAX_TYPE,	
}Compare_type_st;


typedef struct
{
	uint32_t lock_time[3];//

	uint16_t last_state;
	char lock_flag;
	
}alarm_lock_st;

typedef struct
{
	char  cycle_flag;//短周期触发标志。
	uint16_t compress_state;//1\2压缩机状态。
	uint32_t start_time[10];//压缩机1、2的启动时间
	uint16 alarm_timer;
}compress_cycle_alarm_st;

#define  MAX_LOCK_CNT 6
typedef struct
{
	
 	uint8_t compress_high_flag[2]; //0表示压缩机1高压报警 1表示压缩机二高压告警
	uint16_t tem_sensor_fault;
	uint16_t hum_sensor_fault;
 	uint32_t fan_timer; // 1second++
	uint32_t compressor1_timer;//1second++
	uint32_t compressor2_timer;//1second++
	alarm_lock_st  alarm_lock[MAX_LOCK_CNT];
/*
	{0xffffffff,0xffffffff,0xffffffff},//ACL_HI_LOCK1                  0
	{0xffffffff,0xffffffff,0xffffffff},//ACL_HI_LOCK2                 1
	{0xffffffff,0xffffffff,0xffffffff},//ACL_LO_LOCK1                  2
	{0xffffffff,0xffffffff,0xffffffff},//ACL_LO_LOCK2                  3
	{0xffffffff,0xffffffff,0xffffffff},//ACL_EXTMP_LOCK1               4
	{0xffffffff,0xffffffff,0xffffffff},//ACL_EXTMP_LOCK2               5

*/
	compress_cycle_alarm_st cmpress_cycle_alarm[2];
	alarm_acl_status_st alarm_sts[ACL_TOTAL_NUM];
	
}alarm_st;



static alarm_st alarm_inst;
extern	sys_reg_st		g_sys; 


static uint16_t io_calc(uint8_t data,uint8_t refer);
static uint16_t compare_calc(int16_t meter,int16_t min,int16_t max,Compare_type_st type);

static uint16_t alarm_lock(uint16_t alarm_id);

static void   alarm_status_bitmap_op (uint8_t alarm_id,uint8_t option);


//检测函数
static  uint16_t acl00(alarm_acl_status_st* acl_ptr);
static	uint16_t acl01(alarm_acl_status_st* acl_ptr);
static	uint16_t acl02(alarm_acl_status_st* acl_ptr);
static	uint16_t acl03(alarm_acl_status_st* acl_ptr);
static	uint16_t acl04(alarm_acl_status_st* acl_ptr);
static	uint16_t acl05(alarm_acl_status_st* acl_ptr);
static	uint16_t acl06(alarm_acl_status_st* acl_ptr);
static	uint16_t acl07(alarm_acl_status_st* acl_ptr);
//modebus slave hardware fault                   
static 	uint16_t acl08(alarm_acl_status_st* acl_ptr);
//modebus slave communication loss                     
static 	uint16_t acl09(alarm_acl_status_st* acl_ptr);
//NTC  invalid
static 	uint16_t acl10(alarm_acl_status_st* acl_ptr);
//风机类故障                               
static	uint16_t acl11(alarm_acl_status_st* acl_ptr);
static	uint16_t acl12(alarm_acl_status_st* acl_ptr);
static	uint16_t acl13(alarm_acl_status_st* acl_ptr);
static	uint16_t acl14(alarm_acl_status_st* acl_ptr);                       
static	uint16_t acl15(alarm_acl_status_st* acl_ptr);
static	uint16_t acl16(alarm_acl_status_st* acl_ptr);
static	uint16_t acl17(alarm_acl_status_st* acl_ptr);
static	uint16_t acl18(alarm_acl_status_st* acl_ptr);
static	uint16_t acl19(alarm_acl_status_st* acl_ptr);
static	uint16_t acl20(alarm_acl_status_st* acl_ptr);                                    
static	uint16_t acl21(alarm_acl_status_st* acl_ptr);
static	uint16_t acl22(alarm_acl_status_st* acl_ptr);


static	uint16_t acl23(alarm_acl_status_st* acl_ptr);
static	uint16_t acl24(alarm_acl_status_st* acl_ptr);
                                            
                                            
//压缩机1短周期                             
static	uint16_t acl25(alarm_acl_status_st* acl_ptr);
                                            
//压缩机1超时运行                           
static	uint16_t acl26(alarm_acl_status_st* acl_ptr);
                                            
                                           
//压缩机2                                   
                                            
static	uint16_t acl27(alarm_acl_status_st* acl_ptr);
static	uint16_t acl28(alarm_acl_status_st* acl_ptr);
static	uint16_t acl29(alarm_acl_status_st* acl_ptr);
static	uint16_t acl30(alarm_acl_status_st* acl_ptr);
//盘管2                                    
static	uint16_t acl31(alarm_acl_status_st* acl_ptr);
static	uint16_t acl32(alarm_acl_status_st* acl_ptr);
static	uint16_t acl33(alarm_acl_status_st* acl_ptr);
static	uint16_t acl34(alarm_acl_status_st* acl_ptr);
//                                         
static	uint16_t acl35(alarm_acl_status_st* acl_ptr);
static	uint16_t acl36(alarm_acl_status_st* acl_ptr);
static	uint16_t acl37(alarm_acl_status_st* acl_ptr);
static	uint16_t acl38(alarm_acl_status_st* acl_ptr);
static	uint16_t acl39(alarm_acl_status_st* acl_ptr);
static  uint16_t acl40(alarm_acl_status_st* acl_ptr);
static	uint16_t acl41(alarm_acl_status_st* acl_ptr);
static	uint16_t acl42(alarm_acl_status_st* acl_ptr);
static	uint16_t acl43(alarm_acl_status_st* acl_ptr);
static	uint16_t acl44(alarm_acl_status_st* acl_ptr);
static	uint16_t acl45(alarm_acl_status_st* acl_ptr);
static	uint16_t acl46(alarm_acl_status_st* acl_ptr);
static	uint16_t acl47(alarm_acl_status_st* acl_ptr);
static	uint16_t acl48(alarm_acl_status_st* acl_ptr);
static	uint16_t acl49(alarm_acl_status_st* acl_ptr);
static	uint16_t acl50(alarm_acl_status_st* acl_ptr);
static	uint16_t acl51(alarm_acl_status_st* acl_ptr);
static	uint16_t acl52(alarm_acl_status_st* acl_ptr);
static	uint16_t acl53(alarm_acl_status_st* acl_ptr);
static	uint16_t acl54(alarm_acl_status_st* acl_ptr);
static	uint16_t acl55(alarm_acl_status_st* acl_ptr);
static	uint16_t acl56(alarm_acl_status_st* acl_ptr);
static	uint16_t acl57(alarm_acl_status_st* acl_ptr);
static	uint16_t acl58(alarm_acl_status_st* acl_ptr);
static	uint16_t acl59(alarm_acl_status_st* acl_ptr);
static	uint16_t acl60(alarm_acl_status_st* acl_ptr);
static	uint16_t acl61(alarm_acl_status_st* acl_ptr);
static	uint16_t acl62(alarm_acl_status_st* acl_ptr);
static	uint16_t acl63(alarm_acl_status_st* acl_ptr);
static	uint16_t acl64(alarm_acl_status_st* acl_ptr);
static	uint16_t acl65(alarm_acl_status_st* acl_ptr);
static	uint16_t acl66(alarm_acl_status_st* acl_ptr);
static	uint16_t acl67(alarm_acl_status_st* acl_ptr);
static	uint16_t acl68(alarm_acl_status_st* acl_ptr);
static	uint16_t acl69(alarm_acl_status_st* acl_ptr);
static	uint16_t acl70(alarm_acl_status_st* acl_ptr);
static	uint16_t acl71(alarm_acl_status_st* acl_ptr);
static	uint16_t acl72(alarm_acl_status_st* acl_ptr);
static	uint16_t acl73(alarm_acl_status_st* acl_ptr);
static	uint16_t acl74(alarm_acl_status_st* acl_ptr);
static	uint16_t acl75(alarm_acl_status_st* acl_ptr);
static	uint16_t acl76(alarm_acl_status_st* acl_ptr);
static	uint16_t acl77(alarm_acl_status_st* acl_ptr);
static	uint16_t acl78(alarm_acl_status_st* acl_ptr);
static	uint16_t acl79(alarm_acl_status_st* acl_ptr);
static	uint16_t acl80(alarm_acl_status_st* acl_ptr);
static  uint16_t acl81 (alarm_acl_status_st* acl_ptr);
static  uint16_t acl82(alarm_acl_status_st* acl_ptr);
//告警输出仲裁
static void alarm_arbiration(void);
static void alarm_bitmap_op(uint8_t component_bpos, uint8_t action);
static void alarm_bitmap_mask_op(uint8_t component_bpos, uint8_t action);

enum
{
		ALARM_ACL_ID_POS=			0,
		ALARM_ACL_EN_MASK_POS		,	
		ALARM_ACL_RESET_POS     ,
		AlARM_ACL_LEVEL_POS     ,
		ALARM_ACL_DEV_POS      ,
		ALARM_ACL_MAX
};


#define ALARM_FSM_INACTIVE			0x0001	
#define ALARM_FSM_PREACTIVE			0x0002
#define ALARM_FSM_ACTIVE				0x0003	
#define ALARM_FSM_POSTACTIVE		0x0004	
#define ALARM_FSM_ERROR					0x0005	

#define ALARM_ACL_TRIGGERED			0x0001
#define ALARM_ACL_CLEARED			0x0000
#define ALARM_ACL_HOLD              0x0002





//uint16_t alarm_tem_erro,alarm_hum_erro;

static uint16_t (* acl[ACL_TOTAL_NUM])(alarm_acl_status_st*) = 
{
	
//回风和送风报警(温度和湿度)
		acl00,//		ACL_HI_TEMP_RETURN	
		acl01,//		ACL_LO_TEMP_RETURN
		acl02,//		ACL_HI_HUM_RETURN	
		acl03,//		ACL_LO_HUM_RETURN	
		acl04,//		ACL_HI_TEMP_SUPPLY	
		acl05,//		ACL_LO_TEMP_SUPPLY	
		acl06,//		ACL_HI_HUM_SUPPLY
		acl07,//		ACL_LO_HUM_SUPPLY	 

////温湿度传感器测试值超限报警
		acl08,//		ACL_MBM_HARD_FAULT  
		acl09,//		
		acl10,//		ACL_NTC_INVALIDE
////以上两个实现方法可能会修改
////风机类故障报警
		acl11,//ACL_FAN_OVERLOAD1		
		acl12,//ACL_FAN_OVERLOAD2		
		acl13,//ACL_FAN_OVERLOAD3		
		acl14,//ACL_FAN_OVERLOAD4		       
		acl15,//ACL_FAN_OVERLOAD5		
		acl16,//ACL_OUT_FAN_OVERLOAD
		acl17,//ACL_FAN_OT1					
		acl18,//ACL_FAN_OT2					
		acl19,//ACL_FAN_OT3					
		acl20,//ACL_FAN_OT4					
		acl21,//ACL_FAN_OT5					          
		acl22,//ACL_OUT_FAN_OT   
		
//压缩机类故障
//压缩机1
		acl23,//ACL_HI_PRESS1                  
		acl24,//ACL_HI_LOCK1			             
		acl25,//ACL_LO_PRESS1				
		acl26,//ACL_LO_LOCK1				
		acl27,//ACL_EXTMP1				
	  acl28,//ACL_EXTMP_LOCK1		
		acl29,//ACL_SHORT_TERM1		
		acl30,//ACL_COMPRESSOR_OT1
//压缩机2
		acl31,//ACL_HI_PRESS2			
		acl32,//ACL_HI_LOCK2			
		acl33,//ACL_LO_PRESS2			
		acl34,//ACL_LO_LOCK2			
		acl35,//ACL_EXTMP2				
		acl36,//ACL_EXTMP_LOCK2		
		acl37,//ACL_SHORT_TERM2		
		acl38,//ACL_COMPRESSOR_OT2
////加湿器相关报警
		acl39,//ACL_HUM_OCURRENT
////加湿器高水位报警()
	  acl40,//ACL_HUM_HI_LEVEL
////加湿器低水位报警
		acl41,//ACL_HUM_LO_LEVEL
		acl42,//ACL_HUM_OT				,
		
////加热器运行报警1,2,3超时报警
		acl43,//ACL_HEATER_OD
		acl44,//ACL_HEATER_OT1			,
		acl45,//ACL_HEATER_OT2			,
////电源类报警
		acl46,//ACL_POWER_LOSS				
		acl47,//ACL_POWER_EP					
		acl48,//ACL_POWER_HI_FD					
		acl49,//ACL_POWER_LO_FD				
		acl50,//ACL_POWER_A_HIGH
		acl51,//ACL_POWER_B_HIGH
		acl52,//ACL_POWER_C_HIGH
		acl53,//ACL_POWER_A_LOW	
		acl54,//ACL_POWER_B_LOW	
		acl55,//ACL_POWER_C_LOW	
		acl56,//ACL_POWER_A_OP	
		acl57,//ACL_POWER_B_OP	
    acl58,//ACL_POWER_C_OP	
////气流丢失 单独函数实现
    acl59,// ACL_AIR_LOSS
//过滤网超时运行
		acl60,//ACL_FILTER_OT					
////过滤网赌						
    acl61,//ACL_FILTER_CLOG				
//// 远程关机 报警
    acl62,//ACL_REMOTE_SHUT				
////地板积水
    acl63,//ACL_WATER_OVERFLOW		
////群控失败告警(专用函数实现)
    acl64,//ACL_GROUP_CONTROL_FAIL
//变频器故障
		acl65,//ACL_TRAIN_FAULT
		acl66,//ACL_WATER_ELEC_HI
		acl67,//ACL_WATER_ELEC_LO
		acl68,//ACL_SMOKE_ALARM
		acl69,//ACL_USR_ALARM1
		acl70,//ACL_BACK_POWER
//盘管1告警
    acl71,//ACL_COIL_HI_TEM1      
		acl72,//ACL_COIL_VALVE_DEFAULT 
		acl73,//ACL_COIL_BLOCKING      
		acl74,//ACL_FAN_OVERLOAD6 

		acl75,//ACL_FAN_OVERLOAD7     
		acl76,//ACL_FAN_OVERLOAD8
		acl77,//ACL_FAN_OT6     
		acl78,//ACL_FAN_OT7

		acl79,//ACL_FAN_OT8
		acl80,//ACL_WATER_PUMP_HI_LIVEL
		acl81,//ACL_HUM_DEFAULT
		acl82,//ACL_FLOW_DIFF



//用户扩展报警预留后期使用		
        
//		ACL_USR_ALARM1					,
//		ACL_USR_ALARM2					,
//		ACL_USR_ALARM3           ,
};

#define DEV_TYPE_COMPRESSOR 0x0000
#define DEV_TYPE_FAN        0x0400
#define DEV_TYPE_OUT_FAN    0x0800
#define DEV_TYPE_HEATER     0x0c00
#define DEV_TYPE_HUM        0x1000
#define DEV_TYPE_POWER      0x1400
#define DEV_TYPE_TEM_SENSOR 0x1800
#define DEV_TYPE_WATER_PUMP 0x1c00
#define DEV_TYPE_OTHER      0x3c00


const uint16_t ACL_CONF[ACL_TOTAL_NUM][ALARM_ACL_MAX]=
//	id ,en_mask,reless_mask,DEV_type
{		
	  0,			0,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_HI_TEMP_RETURN						
		1,			0,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_LO_TEMP_RETURN						
		2,			0,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_HI_HUM_RETURN						
		3,			0,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_LO_HUM_RETURN							
		4,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_HI_TEMP_SUPPLY						
		5,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_LO_TEMP_SUPPLY						
		6,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_HI_HUM_SUPPLY						
		7,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_TEM_SENSOR,           //ACL_LO_HUM_SUPPLY						
		8,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,                //ACL_MBM_HARD_FAULT						
		9,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,                //ACL_MBM_COM_LOSS
		10,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,                //ACL_NTC_INVALIDE		
	//fan							  	
		11,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,                 //ACL_FAN_OVERLOAD1					
		12,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,                 //ACL_FAN_OVERLOAD2					
		13,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,                 //ACL_FAN_OVERLOAD3		
		14,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,                 //ACL_FAN_OVERLOAD4		
		15,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,                 //ACL_FAN_OVERLOAD5		
		16,			3,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OUT_FAN,             //ACL_OUT_FAN_OVERLOA
		17,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,                 //ACL_FAN_OT1					
		18,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,                 //ACL_FAN_OT2					
		19,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,                 //ACL_FAN_OT3					
		20,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,                 //ACL_FAN_OT4					
		21,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,                 //ACL_FAN_OT5				
		22,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_OUT_FAN,             //ACL_OUT_FAN_OT  
//compressor1	                               
		23,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_HI_PRESS1     
		24,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_HI_LOCK1			
		25,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_LO_PRESS1			
		26,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_LO_LOCK1			
		27,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_EXTMP1				
		28,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_EXTMP_LOCK1			
		29,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_SHORT_TERM1		
		30,			3,		0,  MIOOR_ALARM_LEVEL,     DEV_TYPE_COMPRESSOR,          //ACL_COMPRESSOR_OT1
//compressor2	        	
		31,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_HI_PRESS2							
		32,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_HI_LOCK2								
		33,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_LO_PRESS2						
		34,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_LO_LOCK2				
		35,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_EXTMP2					
		36,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_EXTMP_LOCK2				
		37,			1,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_COMPRESSOR,          //ACL_SHORT_TERM2			
		38,			3,		0,  MIOOR_ALARM_LEVEL,     DEV_TYPE_COMPRESSOR,          //ACL_COMPRESSOR_OT2	
//hum                 
		39,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_HUM,                 //ACL_HUM_OCURRENT				
		40,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_HUM,                //ACL_HUM_HI_LEVEL	
		41,			1,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_HUM,                //ACL_HUM_LO_LEVEL	
		42,			3,    4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_HUM,                //ACL_HUM_OT		
//heater				      	
		43,			1,    4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_HEATER,            //ACL_HEATER_OD					
		44,			3,    0,  MIOOR_ALARM_LEVEL,     DEV_TYPE_HEATER,            //ACL_HEATER_OT1					
		45,			3,    0,  MIOOR_ALARM_LEVEL,     DEV_TYPE_HEATER,						//ACL_HEATER_OT2
//电源类报警          
		                  
		46,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_POWER,//ACL_POWER_LOSS						
		47,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_EP					
		48,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_HI_FD	
		49,     0,    4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_LO_FD	
		50,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_A_HIGH	
		51,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_B_HIGH
		52,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_C_HIGH
		53,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_A_LOW					
		54,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_B_LOW					
		55,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_C_LOW		
		56,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_A_OP
		57,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_B_OP
    58,     0,    4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_POWER,//ACL_POWER_C_OP
		                  
		59,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_AIR_LOSS
		                  
		60,			3,		0,  MIOOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_FILTER_OT
		61,     3,    4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_FILTER_CLOG
		                  
		62,			0,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_REMOTE_SHUT
		                  
		63,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_WATER_OVERFLOW
		64,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_GROUP_CONTROL_FAIL	
		                  
		                  
    65,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_TRAIN_FAULT 
    66,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_HUM,//ACL_WATER_ELEC_HI
    67,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_HUM,//ACL_WATER_ELEC_LO
    68,			3,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_SMOKE_ALARM
    69,			3,		4,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_OTHER,//ACL_USR_ALARM1
    70,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_BACK_POWER
		//盘管告警       
		71,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_COIL_HI_TEM1     
		72,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_COIL_VALVE_DEFAULT
    73,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_COIL_BLOCKING     
    74,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,//ACL_FAN_OVERLOAD6
    75,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN,//ACL_FAN_OVERLOAD7     
    76,			0,		0,  CRITICAL_ALARM_lEVEL,  DEV_TYPE_FAN, //ACL_FAN_OVERLOAD8
    77,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN, //ACL_FAN_OT6     
    78,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN, //ACL_FAN_OT7
		79,     3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN, //ACL_FAN_OT8
    //水盘高水位告警  	
		80,			3,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_OTHER,//ACL_WATER_PUMP_HI_LIVEL
		81,			1,		4,  MAJOR_ALARM_LEVEL,     DEV_TYPE_HUM,//ACL_HUM_DEFAULT		
		//ACL_FLOW_DIFF
		82,			3,		4,  MIOOR_ALARM_LEVEL,     DEV_TYPE_FAN,//ACL_HUM_DEFAULT	

};
/*
  * @brief  alarm data structure initialization
	* @param  none
  * @retval none
  */

static void init_alarm(alarm_st* alarm_spr) 
{
		uint16 i;
		
		//初始ACL 
		for(i=0;i<ACL_TOTAL_NUM;i++)
		{
			
			alarm_spr->alarm_sts[i].timeout =  0;
			alarm_spr->alarm_sts[i].state = ALARM_FSM_INACTIVE;
			alarm_spr->alarm_sts[i].id = ACL_CONF[i][ALARM_ACL_ID_POS];
			alarm_spr->alarm_sts[i].enable_mask = ACL_CONF[i][ALARM_ACL_EN_MASK_POS];
			alarm_spr->alarm_sts[i].reset_mask = ACL_CONF[i][ALARM_ACL_RESET_POS];
			alarm_spr->alarm_sts[i].alarm_level = ACL_CONF[i][AlARM_ACL_LEVEL_POS];
			alarm_spr->alarm_sts[i].dev_type = ACL_CONF[i][ALARM_ACL_DEV_POS];
			alarm_spr->alarm_sts[i].alarm_proc = acl[i];
			alarm_spr->alarm_sts[i].alram_value=0xffff;
		}
		
		//初始化lock类变量
		for(i=0;i<MAX_LOCK_CNT;i++)
		{
			alarm_spr->alarm_lock[i].last_state = ALARM_FSM_INACTIVE;
			alarm_spr->alarm_lock[i].lock_flag = 0;
			alarm_spr->alarm_lock[i].lock_time[0] = 0xffffffff;
			alarm_spr->alarm_lock[i].lock_time[1] = 0xffffffff;
			alarm_spr->alarm_lock[i].lock_time[2] = 0xffffffff;
		}
		//初始化压缩机类 短周期
		//alarm_spr->cmpress_cycle_alarm[0].alarm_timer
		alarm_spr->cmpress_cycle_alarm[0].cycle_flag = 0;
		alarm_spr->cmpress_cycle_alarm[1].cycle_flag = 0;

		alarm_spr->cmpress_cycle_alarm[0].compress_state = COMPRESSOR_FSM_STATE_IDLE;
		alarm_spr->cmpress_cycle_alarm[1].compress_state = COMPRESSOR_FSM_STATE_IDLE;
		
		for(i=0;i<10;i++)
		{
				alarm_spr->cmpress_cycle_alarm[0].start_time[i] = 0xffffffff; 
		    alarm_spr->cmpress_cycle_alarm[1].start_time[i] = 0xffffffff; 
		}
		//高压触发报警初始化
		alarm_spr->compress_high_flag[0] = 0;
		alarm_spr->compress_high_flag[1] = 0;
		//初始化启动定时器
		alarm_spr->fan_timer = 0;
		alarm_spr->compressor1_timer = 0;
		alarm_spr->compressor2_timer = 0;
		
		
}


void alarm_acl_init(void)
{
	uint8_t i;
	// 初始化静态内存分配空间
	chain_init();
	init_alarm(&alarm_inst);
	//初始化手动解除报警
	for(i=0;i<ALARM_TOTAL_WORD;i++)
	{
			g_sys.config.general.alarm_remove_bitmap[i] = 0;
	}
} 

//uint8_t clear_alarm(uint8_t alarm_id)
//{
//		uint8_t byte_offset,bit_offset,i;
//	
//		if(alarm_id == 0xFF)
//		{
//					for(i=0;i<ALARM_TOTAL_WORD;i++)
//					{
//						g_sys.config.general.alarm_remove_bitmap[i] = 0xffff;
//					}
//				
//				return(1);
//		}
//		if(alarm_id < ACL_TOTAL_NUM)
//		{
//				byte_offset = alarm_id >> 4;
//				bit_offset = alarm_id & 0x0f;
//				g_sys.config.general.alarm_remove_bitmap[byte_offset] |= 0x0001<< bit_offset;
//			  return(1);
//		}
//		else
//		{
//				return(0);
//		}			
//}
 
uint8_t clear_alarm(void)
{
		uint8_t i;	

		for(i=0;i<ALARM_TOTAL_WORD;i++)
		{
			g_sys.config.general.alarm_remove_bitmap[i] = 0xffff;
		}
		
		return 1;		
}

static uint8_t get_alarm_remove_bitmap(uint8_t alarm_id)
{
	uint8_t byte_offset,bit_offset;
	if(alarm_id < ACL_TOTAL_NUM)
	{
		
			byte_offset = alarm_id >> 4;
			bit_offset = alarm_id & 0x0f;
			if((g_sys.config.general.alarm_remove_bitmap[byte_offset] >> bit_offset) & 0x0001 )
			{
					return(1);
			}
			else
			{
					return(0);
			}
	}
	else
	{
			return(0);
	}
	
}

static void clear_alarm_remove_bitmap(uint8_t alarm_id)
{
	uint8_t byte_offset,bit_offset;
	
		if(alarm_id < ACL_TOTAL_NUM)
		{
				byte_offset = alarm_id >> 4;
				bit_offset = alarm_id & 0x0f;
				g_sys.config.general.alarm_remove_bitmap[byte_offset] = g_sys.config.general.alarm_remove_bitmap[byte_offset] & (~(0x0001 << bit_offset));
		}
	
}

/**
  * @brief  alarm check list function execution
	* @param  none
  * @retval none
  */
 static void acl_power_on_timer(void)
{
		extern local_reg_st		l_sys;
		if(g_sys.status.dout_bitmap & (0x0001 << DO_COMP1_BPOS))
		{
			 alarm_inst.compressor1_timer++;	
		}
		else
		{
			alarm_inst.compressor1_timer = 0;	
		}
		
		if(g_sys.status.dout_bitmap & (0x0001 << DO_COMP2_BPOS))
		{
			 alarm_inst.compressor2_timer++;	
		}
		else
		{
			alarm_inst.compressor2_timer = 0;	
		}
		
		if(g_sys.status.dout_bitmap & (0x0001 << DO_FAN_BPOS))
		{
			 alarm_inst.fan_timer ++;	
		}
		else
		{
			alarm_inst.fan_timer = 0;	
		}
}

	
void alarm_acl_exe(void)
{
		extern	sys_reg_st		g_sys;
		uint16_t acl_trigger_state;	
		uint16_t i;
		uint16_t c_state;
		uint16_t log_id;
		
		
		acl_power_on_timer();
		
    for(i=0;i<ACL_TOTAL_NUM;i++)	
		{
				//if acl disabled, continue loop
			
				if(((g_sys.config.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)&&(alarm_inst.alarm_sts[i].state == ALARM_FSM_INACTIVE))
				{
						continue;				
				}
			
				acl_trigger_state = acl[i](&alarm_inst.alarm_sts[i]);
				c_state = alarm_inst.alarm_sts[i].state;	
				log_id = alarm_inst.alarm_sts[i].id|(alarm_inst.alarm_sts[i].alarm_level<<8)|alarm_inst.alarm_sts[i].dev_type;
				switch (c_state)
				{
						case(ALARM_FSM_INACTIVE):
						{
							  if(acl_trigger_state == ALARM_ACL_TRIGGERED)
								{
										alarm_inst.alarm_sts[i].timeout = g_sys.config.alarm[i].delay;
										alarm_inst.alarm_sts[i].state = ALARM_FSM_PREACTIVE;
								}
								else if(acl_trigger_state == ALARM_ACL_HOLD)
								{
								 		alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
								}
								else
								{
										;
								}
								
								break;
						}
						case(ALARM_FSM_PREACTIVE):
						{
									 //状态机回到 ALARM_FSM_INACTIVE 状态
								if((g_sys.config.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
								{
										alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;		
								}
								else if(acl_trigger_state == ALARM_ACL_TRIGGERED)
								{
										
									
									 if(alarm_inst.alarm_sts[i].timeout > 0)
										{
												alarm_inst.alarm_sts[i].timeout --;
												alarm_inst.alarm_sts[i].state = ALARM_FSM_PREACTIVE;
										}
										else
										{
												alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
												//yxq
												add_alarmlog_fifo(log_id,ALARM_TRIGER,alarm_inst.alarm_sts[i].alram_value);
												node_append(log_id,alarm_inst.alarm_sts[i].alram_value);
												alarm_status_bitmap_op(alarm_inst.alarm_sts[i].id&0x00ff,1);
										}
								}
								else if(acl_trigger_state == ALARM_ACL_HOLD)
								{
								 		;
								}
								else
								{
										alarm_inst.alarm_sts[i].timeout = 0;
										alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;		
								}							
								break;
						}
						case(ALARM_FSM_ACTIVE):
						{
							 //状态机回到 ALARM_FSM_INACTIVE 状态
								if((g_sys.config.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
								{
										alarm_inst.alarm_sts[i].timeout = 0;
									  alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;		
								}
								else if(acl_trigger_state == ALARM_ACL_TRIGGERED)
								{					
										alarm_inst.alarm_sts[i].timeout = 0;
										alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
								}
								else if(acl_trigger_state == ALARM_ACL_CLEARED)
								{	
									//自动解除报警
									if((g_sys.config.alarm[i].enable_mode & alarm_inst.alarm_sts[i].reset_mask) == ACL_ENMODE_AUTO_RESET_ALARM)
									{
										alarm_inst.alarm_sts[i].timeout =g_sys.config.alarm[i].delay;
										
										alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;	
									}
									else
									{
										alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
									}
								}
								else if(acl_trigger_state == ALARM_ACL_HOLD)
								{
								 		;
								}
								else
								{
									;
								}
								break;
						}
						case(ALARM_FSM_POSTACTIVE):
						{
								 //状态机回到 ALARM_FSM_INACTIVE 状态
								if((g_sys.config.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
								{
											add_alarmlog_fifo(log_id,ALARM_END,alarm_inst.alarm_sts[i].alram_value);												
									     //删除状态节点
											node_delete(log_id);
											alarm_status_bitmap_op(alarm_inst.alarm_sts[i].id&0x00ff,0);
												
											alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
											alarm_inst.alarm_sts[i].timeout = 0;
								}
								else if(acl_trigger_state == ALARM_ACL_CLEARED)//yxq
								{
										
										if(alarm_inst.alarm_sts[i].timeout > 0)
										{
												alarm_inst.alarm_sts[i].timeout --;
												alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;
										}
										else
										{
												//yxq
												add_alarmlog_fifo(log_id,ALARM_END,alarm_inst.alarm_sts[i].alram_value);
//												
//												//删除状态节点
												node_delete(log_id);
												alarm_status_bitmap_op(i,0);
												
												alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
										}
												
								}
								else if(acl_trigger_state == ALARM_ACL_TRIGGERED)
								{
										alarm_inst.alarm_sts[i].timeout = 0;
										alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
								}
								else if(acl_trigger_state == ALARM_ACL_HOLD)
								{
								 		;
								}
								else
								{
									;
								}
								break;
						}
						default://yxq
							{
								alarm_inst.alarm_sts[i].state=ALARM_FSM_INACTIVE;
								
								break;
							}
				}
		}
	
								
		alarm_arbiration();
}
//获取报警位
uint8_t get_alarm_bitmap(uint8_t alarm_id)
{
	uint8_t byte_offset,bit_offset;

	byte_offset = alarm_id>>4;
	bit_offset = alarm_id&0x0f;
	
	if((g_sys.status.alarm_bitmap[byte_offset] >>bit_offset) & (0x0001))
	{
		return(1);
	}
	else
	{
		return(0);
	}
	
}
	
static void alarm_arbiration(void)
{
	uint8_t compress1_alarm=0, compress2_alarm=0,close_dev=0;
	uint8_t index;
	uint8_t fan_default_cnt = 0;

	
//	//压缩机1报警
//				ACL_HI_PRESS1					  , 					
//				ACL_HI_LOCK1					  ,
//				ACL_LO_PRESS1					  ,
//				ACL_LO_LOCK1					  ,
//				ACL_EXTMP1						  ,
//				ACL_EXTMP_LOCK1 				  ,
//	//冷冻水阀(NTC温度模拟量报警)
//				ACL_COIL_HI_TEM1				  ,
//				ACL_COIL_VALVE_DEFAULT			  ,
//				ACL_COIL_BLOCKING				  ,
//				ACL_FAN_OVERLOAD6			  ,
//				
//	//压缩机1报警	
//				ACL_SHORT_TERM1 				,
//外风机故障
if((alarm_inst.compress_high_flag[0])||(get_alarm_bitmap(ACL_HI_PRESS1))||(get_alarm_bitmap(ACL_HI_LOCK1))||
	(get_alarm_bitmap(ACL_LO_PRESS1))||(get_alarm_bitmap(ACL_LO_LOCK1))||
	(get_alarm_bitmap(ACL_EXTMP1))||(get_alarm_bitmap(ACL_EXTMP_LOCK1))||
	(get_alarm_bitmap(ACL_SHORT_TERM1))||(get_alarm_bitmap(ACL_TRAIN_FAULT))||(get_alarm_bitmap(ACL_OUT_FAN_OVERLOAD))||(get_alarm_bitmap(ACL_FAN_OVERLOAD4)))
	{
		compress1_alarm=1;
	}
	else
	{
		compress1_alarm=0;
	}
		
//压缩机2
if((alarm_inst.compress_high_flag[1])||(get_alarm_bitmap(ACL_HI_PRESS2))||(get_alarm_bitmap(ACL_HI_LOCK2))||
	(get_alarm_bitmap(ACL_LO_PRESS2))||(get_alarm_bitmap(ACL_LO_LOCK2))||
	(get_alarm_bitmap(ACL_EXTMP2))||(get_alarm_bitmap(ACL_EXTMP_LOCK2))||
	(get_alarm_bitmap(ACL_SHORT_TERM2))||(get_alarm_bitmap(ACL_OUT_FAN_OVERLOAD))||(get_alarm_bitmap(ACL_FAN_OVERLOAD5)))
	{
		compress2_alarm=1;
	}
	else
	{
		compress2_alarm=0;
	}

	
//关闭设备

			////电源类报警

			//			ACL_POWER_LOSS					,//ABC???????????
			//			ACL_POWER_FD						,	//freqency deviation
			//			ACL_POWER_A_HIGH				,
			//			ACL_POWER_B_HIGH				,
			//			ACL_POWER_C_HIGH				,
			//			ACL_POWER_A_LOW					,
			//			ACL_POWER_B_LOW					,
			//			ACL_POWER_C_LOW					,
			//			ACL_POWER_A_OP					,	//open phase
			//			ACL_POWER_B_OP					,	//open phase
			//			ACL_POWER_C_OP					,	//open phase

			//气流丢失 单独函数实现
			//				ACL_AIR_LOSS				   ,

				// 远程关机 报警
			//				ACL_REMOTE_SHUT 			 ,
				//地板积水
			//				ACL_WATER_OVERFLOW		 ,	
			//ACL_FAN_OVERLOAD1
			//ACL_FAN_OVERLOAD2
			//ACL_FAN_OVERLOAD3
			//烟雾告警
		if(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)
		{
				if((get_alarm_bitmap(ACL_POWER_HI_FD))||(get_alarm_bitmap(ACL_POWER_LO_FD))||
				(get_alarm_bitmap(ACL_POWER_A_HIGH))||(get_alarm_bitmap(ACL_POWER_B_HIGH))||(get_alarm_bitmap(ACL_POWER_C_HIGH))||
				(get_alarm_bitmap(ACL_POWER_A_LOW))||(get_alarm_bitmap(ACL_POWER_B_LOW))||(get_alarm_bitmap(ACL_POWER_C_LOW))||
				(get_alarm_bitmap(ACL_POWER_A_OP))||(get_alarm_bitmap(ACL_POWER_B_OP))||(get_alarm_bitmap(ACL_POWER_C_OP))||
				(get_alarm_bitmap(ACL_AIR_LOSS))||(get_alarm_bitmap(ACL_REMOTE_SHUT))||(get_alarm_bitmap(ACL_WATER_OVERFLOW))||
				(get_alarm_bitmap(ACL_FAN_OVERLOAD1))||(get_alarm_bitmap(ACL_FAN_OVERLOAD2))||(get_alarm_bitmap(ACL_FAN_OVERLOAD3))||
				/*(get_alarm_bitmap(ACL_FAN_OVERLOAD4))||(get_alarm_bitmap(ACL_FAN_OVERLOAD5))||*/(get_alarm_bitmap(ACL_SMOKE_ALARM)))
				{
					close_dev=1;	
				}
				else
				{
					close_dev=0;
				}
		}
		else if(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER)
		{
			if((get_alarm_bitmap(ACL_POWER_HI_FD))||(get_alarm_bitmap(ACL_POWER_LO_FD))||
				(get_alarm_bitmap(ACL_POWER_A_HIGH))||(get_alarm_bitmap(ACL_POWER_B_HIGH))||(get_alarm_bitmap(ACL_POWER_C_HIGH))||
				(get_alarm_bitmap(ACL_POWER_A_LOW))||(get_alarm_bitmap(ACL_POWER_B_LOW))||(get_alarm_bitmap(ACL_POWER_C_LOW))||
				(get_alarm_bitmap(ACL_POWER_A_OP))||(get_alarm_bitmap(ACL_POWER_B_OP))||(get_alarm_bitmap(ACL_POWER_C_OP))||
				(get_alarm_bitmap(ACL_AIR_LOSS))||(get_alarm_bitmap(ACL_REMOTE_SHUT))||(get_alarm_bitmap(ACL_WATER_OVERFLOW))||
				(get_alarm_bitmap(ACL_FAN_OVERLOAD1))||(get_alarm_bitmap(ACL_FAN_OVERLOAD2))||(get_alarm_bitmap(ACL_FAN_OVERLOAD3))||
				/*(get_alarm_bitmap(ACL_FAN_OVERLOAD4))||(get_alarm_bitmap(ACL_FAN_OVERLOAD5))||*/(get_alarm_bitmap(ACL_SMOKE_ALARM)))
				{
					close_dev=1;	
				}
				else
				{
					close_dev=0;
				}
		}
		else if(g_sys.config.general.cool_type == COOL_TYPE_MODULE_MIX)
		{
				if((get_alarm_bitmap(ACL_POWER_HI_FD))||(get_alarm_bitmap(ACL_POWER_LO_FD))||
				(get_alarm_bitmap(ACL_POWER_A_HIGH))||(get_alarm_bitmap(ACL_POWER_B_HIGH))||(get_alarm_bitmap(ACL_POWER_C_HIGH))||
				(get_alarm_bitmap(ACL_POWER_A_LOW))||(get_alarm_bitmap(ACL_POWER_B_LOW))||(get_alarm_bitmap(ACL_POWER_C_LOW))||
				(get_alarm_bitmap(ACL_POWER_A_OP))||(get_alarm_bitmap(ACL_POWER_B_OP))||(get_alarm_bitmap(ACL_POWER_C_OP))||
				(get_alarm_bitmap(ACL_AIR_LOSS))||(get_alarm_bitmap(ACL_REMOTE_SHUT))||(get_alarm_bitmap(ACL_WATER_OVERFLOW))||
				(get_alarm_bitmap(ACL_FAN_OVERLOAD1))||(get_alarm_bitmap(ACL_FAN_OVERLOAD2))||(get_alarm_bitmap(ACL_FAN_OVERLOAD3))||
				/*(get_alarm_bitmap(ACL_FAN_OVERLOAD4))||(get_alarm_bitmap(ACL_FAN_OVERLOAD5))||*/(get_alarm_bitmap(ACL_SMOKE_ALARM)))
				{
					close_dev=1;	
				}
				else
				{
					close_dev=0;
				}
		}
		else if(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND)
		{
				fan_default_cnt = get_alarm_bitmap(ACL_FAN_OVERLOAD1);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD2);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD3);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD4);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD5);
				if((get_alarm_bitmap(ACL_POWER_HI_FD))||(get_alarm_bitmap(ACL_POWER_LO_FD))||
				(get_alarm_bitmap(ACL_POWER_A_HIGH))||(get_alarm_bitmap(ACL_POWER_B_HIGH))||(get_alarm_bitmap(ACL_POWER_C_HIGH))||
				(get_alarm_bitmap(ACL_POWER_A_LOW))||(get_alarm_bitmap(ACL_POWER_B_LOW))||(get_alarm_bitmap(ACL_POWER_C_LOW))||
				(get_alarm_bitmap(ACL_POWER_A_OP))||(get_alarm_bitmap(ACL_POWER_B_OP))||(get_alarm_bitmap(ACL_POWER_C_OP))||
				(get_alarm_bitmap(ACL_AIR_LOSS))||(get_alarm_bitmap(ACL_REMOTE_SHUT))||(get_alarm_bitmap(ACL_WATER_OVERFLOW))||
				(get_alarm_bitmap(ACL_SMOKE_ALARM))||(fan_default_cnt >1))
				{
					close_dev=1;	
				}
				else
				{
					close_dev=0;
				}
		}
		else if(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER)
		{
				fan_default_cnt = get_alarm_bitmap(ACL_FAN_OVERLOAD1);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD2);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD3);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD4);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD5);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD6);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD7);
				fan_default_cnt += get_alarm_bitmap(ACL_FAN_OVERLOAD8);
				if((get_alarm_bitmap(ACL_POWER_HI_FD))||(get_alarm_bitmap(ACL_POWER_LO_FD))||
				(get_alarm_bitmap(ACL_POWER_A_HIGH))||(get_alarm_bitmap(ACL_POWER_B_HIGH))||(get_alarm_bitmap(ACL_POWER_C_HIGH))||
				(get_alarm_bitmap(ACL_POWER_A_LOW))||(get_alarm_bitmap(ACL_POWER_B_LOW))||(get_alarm_bitmap(ACL_POWER_C_LOW))||
				(get_alarm_bitmap(ACL_POWER_A_OP))||(get_alarm_bitmap(ACL_POWER_B_OP))||(get_alarm_bitmap(ACL_POWER_C_OP))||
				(get_alarm_bitmap(ACL_AIR_LOSS))||(get_alarm_bitmap(ACL_REMOTE_SHUT))||(get_alarm_bitmap(ACL_WATER_OVERFLOW))||
				(get_alarm_bitmap(ACL_SMOKE_ALARM))||(fan_default_cnt >=g_sys.config.fan.num))
				{
					close_dev=1;	
				}
				else
				{
					close_dev=0;
				}
		}
		//压缩机1的控制
		if((close_dev)||(compress1_alarm))
		{
			alarm_bitmap_op(DO_COMP1_BPOS,0);
			alarm_bitmap_mask_op(DO_COMP1_BPOS,1);
		}
		else
		{
			alarm_bitmap_op(DO_COMP1_BPOS,1);
			alarm_bitmap_mask_op(DO_COMP1_BPOS,0);
		}

		//压缩机2的控制
		if((close_dev)||(compress2_alarm))
		{
			alarm_bitmap_op(DO_COMP2_BPOS,0);
			alarm_bitmap_mask_op(DO_COMP2_BPOS,1);
		}
		else
		{
			alarm_bitmap_op(DO_COMP2_BPOS,1);
			alarm_bitmap_mask_op(DO_COMP2_BPOS,0);
		}
		//风机控制
		if((close_dev))
		{
			alarm_bitmap_op(DO_FAN_BPOS,0);
			alarm_bitmap_mask_op(DO_FAN_BPOS,1);
		}
		else
		{
			alarm_bitmap_op(DO_FAN_BPOS,1);
			alarm_bitmap_mask_op(DO_FAN_BPOS,0);
		}
		//注水阀控制 高水位告警关闭注水阀
		if((close_dev)||(get_alarm_bitmap(ACL_HUM_HI_LEVEL))||(get_alarm_bitmap(ACL_HUM_LO_LEVEL)))
		{
			alarm_bitmap_op(DO_FILL_BPOS,0);
			alarm_bitmap_mask_op(DO_FILL_BPOS,1);
		}
		else
		{
			alarm_bitmap_op(DO_FILL_BPOS,1);
			alarm_bitmap_mask_op(DO_FILL_BPOS,0);
		}
		//加湿器控制，低水位告警关闭加湿器,加湿器电流过大,加湿器故障
		
		if((close_dev)||(get_alarm_bitmap(ACL_HUM_LO_LEVEL))||(get_alarm_bitmap(ACL_HUM_OCURRENT))||(get_alarm_bitmap(ACL_HUM_DEFAULT)))
		{
				alarm_bitmap_op(DO_HUM_BPOS,0);
				alarm_bitmap_mask_op(DO_HUM_BPOS,1);
				alarm_bitmap_op(DO_FILL_BPOS,0);
				alarm_bitmap_mask_op(DO_FILL_BPOS,1);
				alarm_bitmap_op(DO_DRAIN_BPOS,0);
				alarm_bitmap_mask_op(DO_DRAIN_BPOS,1);
		}
		else
		{
				alarm_bitmap_op(DO_HUM_BPOS,1);
				alarm_bitmap_mask_op(DO_HUM_BPOS,0);
				alarm_bitmap_op(DO_FILL_BPOS,1);
				alarm_bitmap_mask_op(DO_FILL_BPOS,0);
				alarm_bitmap_op(DO_DRAIN_BPOS,1);
				alarm_bitmap_mask_op(DO_DRAIN_BPOS,0);
		}
		//加热器， 加热器过载
		if((close_dev)||(get_alarm_bitmap(ACL_HEATER_OD)))
		{
			
			alarm_bitmap_op(DO_RH1_BPOS,0);
			alarm_bitmap_mask_op(DO_RH1_BPOS,1);

			alarm_bitmap_op(DO_RH2_BPOS,0);
			alarm_bitmap_mask_op(DO_RH2_BPOS,1);

		}
		else
		{
			alarm_bitmap_op(DO_RH1_BPOS,1);
			alarm_bitmap_mask_op(DO_RH1_BPOS,0);

			alarm_bitmap_op(DO_RH2_BPOS,1);
			alarm_bitmap_mask_op(DO_RH2_BPOS,0);

		}
		
		// 状态更新
				//群控失败
		if(get_alarm_bitmap(ACL_GROUP_CONTROL_FAIL))
		{
				sys_set_remap_status(WORK_MODE_STS_REG_NO,TEAM_STS_BPOS, 1);
		}
		else
		{
				sys_set_remap_status(WORK_MODE_STS_REG_NO,TEAM_STS_BPOS, 0);
		}
    //phase protect
//		if( get_alarm_bitmap(ACL_POWER_EP))
//		{
//				req_bitmap_op(DO_PHASE_P_BPOS,0);
//				req_bitmap_op(DO_PHASE_N_BPOS,1);
//		}
//		else
//		{
//				req_bitmap_op(DO_PHASE_P_BPOS,1);
//				req_bitmap_op(DO_PHASE_N_BPOS,0);
//		}
		 //关闭公共告警
		alarm_bitmap_op(DO_ALARM_BPOS,0);
		alarm_bitmap_mask_op(DO_ALARM_BPOS,1);
			//开启 公共报警开关
		for(index=0;index<ACL_TOTAL_NUM;index++)
		{
				if(((g_sys.config.alarm[index].enable_mode & alarm_inst.alarm_sts[index].enable_mask) == ACL_ENMODE_ENABLE)||
					((g_sys.config.alarm[index].enable_mode & alarm_inst.alarm_sts[index].enable_mask) == ACL_ENMODE_SUPPRESS))
				{
					if(get_alarm_bitmap(index))//报警存在	
					{
						//开启公共报警
						
						alarm_bitmap_op(DO_ALARM_BPOS,1);
						alarm_bitmap_mask_op(DO_ALARM_BPOS,1);
						break;
					}
					
				}
		}
		
	
}

//运行时间计算 
//返回值时间单位是天
static uint16_t dev_runingtime(uint16_t low,uint16_t high)
{
		uint16_t runing_day;
		uint32_t run_time;
		
		run_time = high;
		run_time = (run_time<<16) + low;
		run_time = run_time >>12;
		runing_day = run_time/24;
	
		return(runing_day);
}


//DO_FAN_BPOS//offset
//报警位操作

static void alarm_bitmap_op(uint8_t component_bpos, uint8_t action)
{
		extern local_reg_st l_sys;
		if(action == 0)
		{
				l_sys.bitmap[BITMAP_ALARM] &= ~(0x0001<<component_bpos);
		}
		else
		{
				l_sys.bitmap[BITMAP_ALARM] |= (0x0001<<component_bpos);
		}
}

static void alarm_bitmap_mask_op(uint8_t component_bpos, uint8_t action)
{
		extern local_reg_st l_sys;
		if(action == 0)
		{
				l_sys.bitmap[BITMAP_MASK] &= ~(0x0001<<component_bpos);
		}
		else
		{
				l_sys.bitmap[BITMAP_MASK] |= (0x0001<<component_bpos);
		}
}


static void   alarm_status_bitmap_op (uint8_t alarm_id,uint8_t option)
{
		uint8_t byte_offset,bit_offset;
		
		byte_offset = alarm_id >> 4;
		bit_offset = alarm_id &0x0f;
		if(option == 1)
		{
				g_sys.status.alarm_bitmap[byte_offset] |= (0x0001<<bit_offset);
		}
		else
		{
				g_sys.status.alarm_bitmap[byte_offset] &= ~(0x0001<<bit_offset);
		}
		
}

/**
  * @brief  alarm 0 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */



static uint16_t io_calc(uint8_t data,uint8_t refer)
{
	
	if(data==refer)
	{
			
		return 1;
	}
	else
	{
		
		return 0;
	}
}

/**
  * @brief  alarm 1 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */

//模拟量报警检测表

//const compare_st  compare_table[]=
//{  //数据测试值 ，报警门限低， ，报警门限高，判断方法
//		{&g_sys.config.algorithm.supply_air_temp,NULL,&g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param,COMPARE_MAX}//supply_air_temp alarm
//		
//};

// 模拟量抱紧检测函数
static uint16_t compare_calc(int16_t meter,int16_t min,int16_t max,Compare_type_st type)
{
	
		if(type==THR_MAX_TYPE)//大于最大门限触发
		{
			if(meter > max)
			{
				return(1);
			}
			else
			{
				return(0);
			}
		}
		else if(type==SML_MIN_TYPE)//小于最小门限触发
		{
			if(meter<min)
			{
				return(1);
			}
			else
			{
				return(0);
			}
		}
		else if(type==IN_MIN_MAX_TYPE)//在区间内报警
		{
			if((meter > min)&&(meter < max))
			{
				return(1);
			}
			else
			{
				return(0);
			}
			
		}
		else//在区间以外报警
		{
			if((meter<min)||(meter>max))
			{
				return(1);
			}
			else
			{
				
				return(0);
			}
			
		}
	
}

/**
  * @brief  alarm 2 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */
  //锁死类报警


static uint16_t alarm_lock(uint16_t alarm_id)
{
	uint8_t index=0xff;
	
	switch(alarm_id)
	{
		case ACL_HI_LOCK1://高压1报警
			index = 0;
		break;
		case  ACL_HI_LOCK2://高压2报警
			index = 1;
		break;
		case ACL_LO_LOCK1://低压1报警
			index = 2;
		break;
		case ACL_LO_LOCK2://低压2报警
			index=3;
		break;
		case ACL_EXTMP_LOCK1://排气管1温度报警
			index = 4;
		break;
		case ACL_EXTMP_LOCK2://排气管2温度报警
			index = 5;
		break;
		default:
			index = 0xff;
		break;
			
	}
	
	if(index != 0xff)
	{
			//解除锁定报警
			if(get_alarm_remove_bitmap(alarm_id) == 1)
			{
					alarm_inst.alarm_lock[index].lock_time[0] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_time[1] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_time[2] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_flag =0;
					
				//	clear_alarm_remove_bitmap(alarm_id);
			}
			
			if(alarm_inst.alarm_lock[index].lock_flag)
			{
					return(ALARM_ACL_TRIGGERED);
			}
			if(alarm_inst.alarm_lock[index].lock_time[0] != 0xffffffff)
			{
					if((alarm_inst.alarm_lock[index].lock_time[2] - alarm_inst.alarm_lock[index].lock_time[0]) <= 3600)
					{
							alarm_inst.alarm_lock[index].lock_flag = 1;
							return(ALARM_ACL_TRIGGERED);
					}	
			}
   }
	return(ALARM_ACL_CLEARED);
}




static  uint8_t acl_clear(alarm_acl_status_st* acl_ptr)
{
		if((get_alarm_remove_bitmap(acl_ptr->id) == 1)&&(ALARM_FSM_ACTIVE ==acl_ptr->state))
		{
				acl_ptr->state = ALARM_FSM_POSTACTIVE;
				acl_ptr->timeout = 0;
				clear_alarm_remove_bitmap(acl_ptr->id);
				return(1);
		}
		clear_alarm_remove_bitmap(acl_ptr->id);
		return(0);
}
static uint16_t acl_get_pwr_sts(void)
{
			return(1);
}
//回风，送风温湿度报警



//ACL_HI_TEMP_RETURN

static  uint16_t acl00(alarm_acl_status_st* acl_ptr)
{
	extern team_local_st team_local_inst;
	int16_t max;
	int16_t meter;
	
	//参数确定
	// 冷启动延时
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param2)
//	{
//			acl_ptr->state = ALARM_FSM_INACTIVE;
//			return(ALARM_ACL_HOLD);
//	}

	//modbus_dev
	if(((g_sys.config.dev_mask.mb_comp&0x000F)==0)&&((g_sys.config.dev_mask.ain&0x0030)==0) )
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.return_air_temp;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
		(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
		max = g_sys.config.alarm[ACL_HI_TEMP_RETURN].alarm_param;
	}
	else
	{
		max = team_local_inst.team_config[TEAM_CONF_ALARM_HI_TEMP_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	
	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}


//ACL_LO_TEMP_RETURN
static  uint16_t acl01(alarm_acl_status_st* acl_ptr)
{
	extern team_local_st team_local_inst;
	int16_t meter;
	int16_t min;
	//参数确定
		// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	// 冷启动延时
	
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param2)
//	{
//			acl_ptr->state = ALARM_FSM_INACTIVE;
//			return(ALARM_ACL_HOLD);
//	}
	//modbus_dev
	if(((g_sys.config.dev_mask.mb_comp&0x000F)==0)&&((g_sys.config.dev_mask.ain&0x0030)==0) )
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.return_air_temp;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}	
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			min = g_sys.config.alarm[ACL_LO_TEMP_RETURN].alarm_param;
	}
	else
	{
		 min = team_local_inst.team_config[TEAM_CONF_ALARM_LO_TEMP_PARAM] ;
	}

	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;

	return(compare_calc( meter,min,0,SML_MIN_TYPE));
	
}

//ACL_HI_HUM_RETURN
static  uint16_t acl02(alarm_acl_status_st*acl_ptr)
{
	extern team_local_st team_local_inst;
	uint16_t meter;
	uint16_t max;
	//参数确定
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	// 冷启动延时
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param2)
//	{
//			acl_ptr->state = ALARM_FSM_INACTIVE;
//			return(ALARM_ACL_HOLD);
//	}
	if((g_sys.config.dev_mask.mb_comp&0x000F)==0)
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.return_air_hum;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			max = g_sys.config.alarm[ACL_HI_HUM_RETURN].alarm_param;;
	}
	else
	{
			max = team_local_inst.team_config[TEAM_CONF_ALARM_HI_HUM_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}

//ACL_LO_HUM_RETURN

static  uint16_t acl03(alarm_acl_status_st*acl_ptr)
{
	extern team_local_st team_local_inst;
	uint16_t meter;
	uint16_t min;
	//参数确定
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	// 冷启动延时
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param2)
//	{
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
	if((g_sys.config.dev_mask.mb_comp&0x000F)==0)
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.return_air_hum;	
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			min = g_sys.config.alarm[ACL_LO_HUM_RETURN].alarm_param;
	}
	else
	{
			min = team_local_inst.team_config[TEAM_CONF_ALARM_LO_HUM_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,min,0,SML_MIN_TYPE));
	
}

//ACL_HI_TEMP_SUPPLY

static  uint16_t acl04(alarm_acl_status_st* acl_ptr)
{
	extern team_local_st team_local_inst;
	int16_t max;
	int16_t meter;
	//参数确定
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	// 冷启动延时
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_HI_TEMP_SUPPLY].alarm_param2)
//	{
//	
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
	if(((g_sys.config.dev_mask.mb_comp&0x0030)==0)&&((g_sys.config.dev_mask.ain&0x00C0)==0) )
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.supply_air_temp;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}		
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
		(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
		max = g_sys.config.alarm[ACL_HI_TEMP_SUPPLY].alarm_param;
	}
	else
	{
		max = team_local_inst.team_config[TEAM_CONF_ALARM_HI_TEMP_PARAM] ;
	}	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}

//ACL_LO_TEMP_SUPPLY
static  uint16_t acl05(alarm_acl_status_st* acl_ptr)
{
	extern team_local_st team_local_inst;
	int16_t meter;
	int16_t min;
	//参数确定
	// 冷启动延时
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_LO_TEMP_SUPPLY].alarm_param2)
//	{
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}	
	//dev
	if(((g_sys.config.dev_mask.mb_comp&0x0030)==0)&&((g_sys.config.dev_mask.ain&0x00C0)==0) )
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.supply_air_temp;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}		
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			min = g_sys.config.alarm[ACL_LO_TEMP_SUPPLY].alarm_param;
	}
	else
	{
		 min = team_local_inst.team_config[TEAM_CONF_ALARM_LO_TEMP_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,min,0,SML_MIN_TYPE));
	
}



//ACL_HI_HUM_SUPPLY
static  uint16_t acl06(alarm_acl_status_st* acl_ptr)
{
	extern team_local_st team_local_inst;
	uint16_t meter;
	uint16_t max;
	//参数确定
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_HI_HUM_SUPPLY].alarm_param2)
//	{
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
	//de
	if((g_sys.config.dev_mask.mb_comp&0x0030)==0)
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.supply_air_hum;
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			max = g_sys.config.alarm[ACL_HI_HUM_SUPPLY].alarm_param;
	}
	else
	{
			max = team_local_inst.team_config[TEAM_CONF_ALARM_HI_HUM_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}
////ACL_LO_HUM_SUPPLY
static  uint16_t acl07(alarm_acl_status_st*acl_ptr)
{
	extern team_local_st team_local_inst;
	uint16_t meter;
	uint16_t min;
	
	// 解除 报警
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_LO_HUM_SUPPLY].alarm_param2)
//	{
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
	if((g_sys.config.dev_mask.mb_comp&0x0030)==0)
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = g_sys.status.sys_tem_hum.supply_air_hum;	
	if(meter == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(((team_local_inst.team_table[g_sys.config.team.addr-1][TEAM_TAB_STATUS]&(0x0001<<TEAM_STS_CONFD_BPOS))==0)||
	(g_sys.config.team.team_en == 0)||(g_sys.config.team.addr >(team_local_inst.team_config[TEAM_CONF_CNT]&0x00ff)))
	{
			min = g_sys.config.alarm[ACL_LO_HUM_SUPPLY].alarm_param;
	}
	else
	{
			min = team_local_inst.team_config[TEAM_CONF_ALARM_LO_HUM_PARAM] ;
	}
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	return(compare_calc( meter,min,0,SML_MIN_TYPE));
	
}
//modbus_slave hardware fault
static  uint16_t acl08(alarm_acl_status_st*acl_ptr)
{
		uint8_t req,index;
		
		req =0;
		// 解除 报警
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		//modbus_dev_default
		for(index=RETURN_TEM_HUM_SENSOR_BPOS; index<=POWER_MODULE_BPOS;index++)
		{
					if(((g_sys.config.dev_mask.mb_comp) & (0X01<<index))&&(sys_get_mbm_online(index) == 1))
					{		
									if(g_sys.status.mbm.tnh[index].dev_sts & MBM_DEV_STS_DEFAULT)
									{
												sys_set_remap_status(SENSOR_STS_REG_NO,index,1);
											
												req =1;
									}
									else
									{
												sys_set_remap_status(SENSOR_STS_REG_NO,index,0);;
									}   	
									
					}
			}
				
		
		acl_ptr->alram_value = g_sys.status.status_remap[SENSOR_STS_REG_NO];
	  return(req);	
}

// modus_slave comunication fault
static  uint16_t acl09(alarm_acl_status_st* acl_ptr)
{
		uint8_t req;
		
		//uint16_t HUM_erro=0;
		req = 0;
		// 解除 报警
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		
		if(g_sys.status.status_remap[MBM_COM_STS_REG_NO] != g_sys.config.dev_mask.mb_comp)
		{
				req = 1;
		}
		acl_ptr->alram_value = g_sys.status.status_remap[MBM_COM_STS_REG_NO]; 
		return(req);

}

// ACL_NTC_INVALID
static  uint16_t acl10(alarm_acl_status_st* acl_ptr)
{
		int16_t min;
		int16_t max;
		uint8_t req;
		int16_t meter;
		
		// 解除 报警
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		
		max = ACL_TEM_MAX;
		min = ACL_TEM_MIN;
		
		req =0;
	//TEM_SENSOR
		alarm_inst.tem_sensor_fault = 0;	
		
		 //NTC
		 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC1_BPOS))
		 {
		 	  
				meter =   g_sys.status.ain[DEV_AIN_RETUREN_NTC1_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
						sys_set_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC1_FAULT_BPOS,1);
				    req =1;
				}
				else
				{
				    sys_set_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC1_FAULT_BPOS,0);
				}   		 	 
		 }
		 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_RETUREN_NTC2_BPOS))
		 {
				meter =   g_sys.status.ain[DEV_AIN_RETUREN_NTC2_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
						sys_set_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC2_FAULT_BPOS,1);	
						req =1;
				}
				else
				{
						sys_set_remap_status(SENSOR_STS_REG_NO,DEV_RETUREN_NTC2_FAULT_BPOS,0);
				}   		
					
		 }
		if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC1_BPOS))
		 {
				meter =   g_sys.status.ain[DEV_AIN_SUPPLY_NTC1_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
						sys_set_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC1_FAULT_BPOS,1);
				    req =1;
				}
				else
				{
				    sys_set_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC1_FAULT_BPOS,0);
				}   	
		 }
		 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_SUPPLY_NTC2_BPOS))
		 {
				meter =   g_sys.status.ain[DEV_AIN_SUPPLY_NTC2_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
						 sys_set_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC2_FAULT_BPOS,1);	
						 req =1;
				}
				else
				{
						 sys_set_remap_status(SENSOR_STS_REG_NO,DEV_SUPPLY_NTC2_FAULT_BPOS,0);
				}      
		 }

		 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_CIOL_NTC1_BPOS))
		 {
				meter =   g_sys.status.ain[DEV_AIN_CIOL_NTC1_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
			      sys_set_remap_status(SENSOR_STS_REG_NO,DEV_CIOL_NTC1_FAULT_BPOS,1);	
				    req =1;
				}
				else
				{
						sys_set_remap_status(SENSOR_STS_REG_NO,DEV_CIOL_NTC1_FAULT_BPOS,0);
				}    
		 	  
		 }
		 if((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_CIOL_NTC2_BPOS))
		 {
				meter =   g_sys.status.ain[DEV_AIN_CIOL_NTC2_BPOS];
				if(compare_calc( meter,min,max,OUT_MIN_MAX_TYPE))
				{
							sys_set_remap_status(SENSOR_STS_REG_NO,DEV_CIOL_NTC2_FAULT_BPOS,1);	
							req =1;
				}
				else
				{
							sys_set_remap_status(SENSOR_STS_REG_NO,DEV_CIOL_NTC2_FAULT_BPOS,0);
				}       
		 }
		 acl_ptr->alram_value = g_sys.status.status_remap[SENSOR_STS_REG_NO]; 
			return(req);

}
//ACL_FAN_OVERLOAD1

static	uint16_t acl11(alarm_acl_status_st* acl_ptr)
{
		uint8_t data;
		
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_HOLD);
		}
		data = sys_get_di_sts(DI_FAN01_OD_BPOS);
		return(io_calc( data,IO_CLOSE));
}
//ACL_FAN_OVERLOAD2

static	uint16_t acl12(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	data = sys_get_di_sts(DI_FAN02_OD_BPOS);	
	return(io_calc( data,IO_CLOSE));
}

//ACL_FAN_OVERLOAD3

static	uint16_t acl13(alarm_acl_status_st*acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	data = sys_get_di_sts(DI_FAN03_OD_BPOS);	
		
	return(io_calc( data,IO_CLOSE));	
}

//ACL_FAN_OVERLOAD4

static	uint16_t acl14(alarm_acl_status_st*acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	data = sys_get_di_sts(DI_FAN04_OD_BPOS);	
		
	return(io_calc( data,IO_CLOSE));	
}
	//ACL_FAN_OVERLOAD5

static	uint16_t acl15(alarm_acl_status_st*acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	data = sys_get_di_sts(DI_FAN05_OD_BPOS);	
		
	return(io_calc( data,IO_CLOSE));	
}
//ACL_OUT_FAN_OVERLOAD
static	uint16_t acl16(alarm_acl_status_st*acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	data = sys_get_di_sts(DI_OSD_OUT_FAN_BPOS);	
		
	return(io_calc( data,IO_CLOSE));	
}
//风机运行超时
//ACL_FAN_OT1
static	uint16_t acl17(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN_BPOS].low,g_sys.status.run_time[DO_FAN_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT1].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}
//ACL_FAN_OT2
static	uint16_t acl18(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN2_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN2_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT2].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}
//ACL_FAN_OT3
static	uint16_t acl19(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN3_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN3_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT3].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}
//ACL_FAN_OT4
static	uint16_t acl20(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN4_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN4_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT4].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}

//ACL_FAN_OT5
static	uint16_t acl21(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN5_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN5_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT5].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}

//ACL_OUT_FAN_OT
static	uint16_t acl22(alarm_acl_status_st*acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}

		run_time = dev_runingtime(g_sys.status.run_time[DO_OUT_FAN_DUMMY_BPOS].low,g_sys.status.run_time[DO_OUT_FAN_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_OUT_FAN_OT].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}




//		alarm_io, //ACL_HI_PRESS1
static	uint16_t acl23(alarm_acl_status_st* acl_ptr)
{
		uint8_t data;
		time_t now;
		uint8_t index;

		index=0;
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}
		
		data = sys_get_di_sts(DI_HI_PRESS1_BPOS);	
		data = io_calc( data,IO_CLOSE);
		
		if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
		{
			if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
				 && (acl_ptr->state == ALARM_FSM_ACTIVE))//从预激活，到激活。
				{
					get_local_time(&now);
					alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
					alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
					alarm_inst.alarm_lock[index].lock_time[2] = now;
				}
			alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
		}

		if(data)
		{
			alarm_inst.compress_high_flag[index] = 1;
		}
		else
		{
			alarm_inst.compress_high_flag[index] = 0;
		}
		
		return(data);	
}
//		alarm_lock,//ACL_HI_LOCK1
static	uint16_t acl24(alarm_acl_status_st* acl_ptr)
{	
		uint8_t req;
	
		//remove loc
		req = alarm_lock(ACL_HI_LOCK1);	
		acl_clear(acl_ptr);
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
		return (req);
	
}
//		alarm_io,//ACL_LO_PRESS1
static	uint16_t acl25(alarm_acl_status_st* acl_ptr)
{
	extern local_reg_st 	l_sys;
	uint8_t data=0;
	time_t now;
	uint8_t index;
	index = 2;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
	{
		if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
			 && (acl_ptr->state == ALARM_FSM_ACTIVE))//从预激活，到激活。
			{
				get_local_time(&now);
				alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
				alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
				alarm_inst.alarm_lock[index].lock_time[2] = now;
			}
		alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
	}
	// 冷启动延时
	if((alarm_inst.compressor1_timer < g_sys.config.alarm[ACL_LO_PRESS1].alarm_param)&&(alarm_inst.compressor1_timer != 0))
	{
			//压缩机已经运行
			
			return(ALARM_ACL_CLEARED);
			
	}
	
	
	data = sys_get_di_sts(DI_LO_PRESS1_BPOS);	
	data = io_calc( data,IO_CLOSE);
	
	return(data);		
}
//		alarm_lock,//ACL_LO_LOCK1
static	uint16_t acl26(alarm_acl_status_st* acl_ptr)
{
		uint8_t req;
		
		//remove loc
		req = alarm_lock(ACL_LO_LOCK1);	
		
		acl_clear(acl_ptr);
	
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
		
		return (req);
	
}
////		alarm_io,//ACL_EXTMP1
static	uint16_t acl27(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	time_t now;
	uint8_t index;
	
	index = 4;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
	{
			return(ALARM_ACL_CLEARED);
	}
	data = sys_get_di_sts(DI_EXT_TEMP1_BPOS);
	data = io_calc( data,IO_CLOSE);
	
	if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
	{
			if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
				 && (acl_ptr->state == ALARM_FSM_ACTIVE))//从预激活，到激活。
				{
					get_local_time(&now);
					alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
					alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
					alarm_inst.alarm_lock[index].lock_time[2] = now;
				}
			alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
	}
	
	return(data);	
}
////		alarm_lock,//ACL_EXTMP_LOCK1
static	uint16_t acl28(alarm_acl_status_st* acl_ptr)
{
		uint8_t req;
		//remove loc
		req = alarm_lock(ACL_EXTMP_LOCK1);	
		
		acl_clear(acl_ptr);
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
		
		return (req);
}





////压缩机1短周期
////ACL_SHORT_TERM1	
static	uint16_t acl29(alarm_acl_status_st* acl_ptr)
{
	extern local_reg_st 	l_sys;
	uint8_t index=0,i;
	time_t now;
	
	
	if(acl_clear(acl_ptr))
	{
			for(i=0;i<10;i++)
			{
					alarm_inst.cmpress_cycle_alarm[index].start_time[i] = 0xffffffff;
			}
			alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
			alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 0;
			return(ALARM_ACL_CLEARED);
	}
	
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
	{
			return(ALARM_ACL_CLEARED);
	}	
	
	if(alarm_inst.cmpress_cycle_alarm[index].compress_state != ((g_sys.status.dout_bitmap >> DO_COMP1_BPOS)&0x0001))
	{
		if((alarm_inst.cmpress_cycle_alarm[index].compress_state == 0)&& //开压缩机判断
			(((g_sys.status.dout_bitmap >> DO_COMP1_BPOS)&0x0001) == 1))	
		{
			get_local_time(&now);
			alarm_inst.cmpress_cycle_alarm[index].start_time[0] = alarm_inst.cmpress_cycle_alarm[index].start_time[1];
			alarm_inst.cmpress_cycle_alarm[index].start_time[1] = alarm_inst.cmpress_cycle_alarm[index].start_time[2];
			alarm_inst.cmpress_cycle_alarm[index].start_time[2] = alarm_inst.cmpress_cycle_alarm[index].start_time[3];
			alarm_inst.cmpress_cycle_alarm[index].start_time[3] = alarm_inst.cmpress_cycle_alarm[index].start_time[4];
			alarm_inst.cmpress_cycle_alarm[index].start_time[4] = alarm_inst.cmpress_cycle_alarm[index].start_time[5];
			alarm_inst.cmpress_cycle_alarm[index].start_time[5] = alarm_inst.cmpress_cycle_alarm[index].start_time[6];
			alarm_inst.cmpress_cycle_alarm[index].start_time[6] = alarm_inst.cmpress_cycle_alarm[index].start_time[7];
			alarm_inst.cmpress_cycle_alarm[index].start_time[7] = alarm_inst.cmpress_cycle_alarm[index].start_time[8];
			alarm_inst.cmpress_cycle_alarm[index].start_time[8] = alarm_inst.cmpress_cycle_alarm[index].start_time[9];
			alarm_inst.cmpress_cycle_alarm[index].start_time[9] = now;

			if((alarm_inst.cmpress_cycle_alarm[index].start_time[0] != 0xffffffff)&&
				((now-alarm_inst.cmpress_cycle_alarm[index].start_time[0]) <= 3600))//1个小时内发生10次压缩机重新启动
			{
				alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 1800;
				alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 1;
			}
				
		}
		//状态跟新
		alarm_inst.cmpress_cycle_alarm[index].compress_state = ((g_sys.status.dout_bitmap >> DO_COMP1_BPOS)&0x0001);
		
	}
	if(alarm_inst.cmpress_cycle_alarm[index].cycle_flag)
	{
		if(alarm_inst.cmpress_cycle_alarm[index].alarm_timer > 0 )
		{
			alarm_inst.cmpress_cycle_alarm[index].alarm_timer--;
			if((get_alarm_bitmap(ACL_HI_TEMP_RETURN)) || (get_alarm_bitmap(ACL_HI_TEMP_SUPPLY)))//高温报警，自动解锁
			{
					alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
					alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 0;
				  return(0);
			}
			else
			{
				  return(1);
			}
		}
		else//延时结束自动解锁
		{
				alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
				return(0);
		}
	}
	else
	{
		   return(0);
	}

}

//ACL_COMPRESSOR_OT1			,

static	uint16_t acl30(alarm_acl_status_st* acl_ptr)
{	
		uint32_t run_time;
		int16_t max;
		//参数确定
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() == 0)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}	
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_COMP1_BPOS].low,g_sys.status.run_time[DO_COMP1_BPOS].high);
		max = g_sys.config.alarm[ACL_COMPRESSOR_OT1].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));	
}

//压缩机2




//		alarm_io, //ACL_HI_PRESS2
static	uint16_t acl31(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	time_t now;
	uint8_t index;

	index=1;
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
				return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type != COOL_TYPE_MODULE_WIND))
	{
			return(ALARM_ACL_CLEARED);
	}
	data = sys_get_di_sts(DI_HI_PRESS2_BPOS);
	data = io_calc( data,IO_CLOSE);
	
	if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
	{
		if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
			 && (acl_ptr->state == ALARM_FSM_ACTIVE))//????,????
			{
				get_local_time(&now);
				alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
				alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
				alarm_inst.alarm_lock[index].lock_time[2] = now;
			}
		alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
	}

	if(data)
	{
		alarm_inst.compress_high_flag[index] = 1;
	}
	else
	{
		alarm_inst.compress_high_flag[index] = 0;
	}
	return(data);	
}
//		alarm_lock,//ACL_HI_LOCK2
static	uint16_t acl32(alarm_acl_status_st* acl_ptr)
{
		
		uint8_t req;
			//remove loc
		req = alarm_lock(ACL_HI_LOCK2);	
		
		acl_clear(acl_ptr);
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}	
		
		return (req);
	
}
//		alarm_io,//ACL_LO_PRESS2
static	uint16_t acl33(alarm_acl_status_st* acl_ptr)
{
	uint8_t data=0;
	time_t now;
	uint8_t index;
	extern local_reg_st  l_sys;	

	// 冷启动延时
	index = 3;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
				return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type != COOL_TYPE_MODULE_WIND))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
	{
			if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
				 && (acl_ptr->state == ALARM_FSM_ACTIVE))//????,????
			{
					get_local_time(&now);
					alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
					alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
					alarm_inst.alarm_lock[index].lock_time[2] = now;
			}
			alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
	}
	
	if((alarm_inst.compressor2_timer < g_sys.config.alarm[ACL_LO_PRESS2].alarm_param)&&(alarm_inst.compressor2_timer != 0))
	{
		//compressor2 power_on
			
				return(ALARM_ACL_CLEARED);
			
	}

	
	
	data = 	data = sys_get_di_sts(DI_LO_PRESS2_BPOS);
	data = io_calc( data,IO_CLOSE);
	
	
	return(data);		
}
//		alarm_lock,//ACL_LO_LOCK2
static	uint16_t acl34(alarm_acl_status_st* acl_ptr)
{
		uint8_t req;
		//remove loc
		req = alarm_lock(ACL_LO_LOCK2);	
		
		acl_clear(acl_ptr);
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
		
		return (req);
}
//		alarm_io,//ACL_EXTMP2
static	uint16_t acl35(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	time_t now;
	uint8_t index;

	index = 5;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type != COOL_TYPE_MODULE_WIND))
	{
			return(ALARM_ACL_CLEARED);
	}
	data = sys_get_di_sts(DI_EXT_TEMP2_BPOS);
	data = io_calc( data,IO_CLOSE);
	
	
	if(alarm_inst.alarm_lock[index].last_state != acl_ptr->state)
	{
			if((alarm_inst.alarm_lock[index].last_state == ALARM_FSM_PREACTIVE)
			&& (acl_ptr->state == ALARM_FSM_ACTIVE))//????,????
			{
					get_local_time(&now);
					alarm_inst.alarm_lock[index].lock_time[0] = alarm_inst.alarm_lock[index].lock_time[1];
					alarm_inst.alarm_lock[index].lock_time[1] = alarm_inst.alarm_lock[index].lock_time[2];
					alarm_inst.alarm_lock[index].lock_time[2] = now;
			}
			alarm_inst.alarm_lock[index].last_state = acl_ptr->state;
	}

	return(data);	
}
//		alarm_lock,//ACL_EXTMP_LOCK2
static	uint16_t acl36(alarm_acl_status_st* acl_ptr)
{
		uint8_t req;
		//remove loc
		req = alarm_lock(ACL_EXTMP_LOCK2);	
		
		acl_clear(acl_ptr);
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
		
		
		return (req);

}



//压缩机短周期

static	uint16_t acl37(alarm_acl_status_st* acl_ptr)
{
	extern local_reg_st 	l_sys;
	uint8_t index=1,i;
	time_t now;
	if(acl_clear(acl_ptr))
	{
			for(i=0;i<10;i++)
			{
					alarm_inst.cmpress_cycle_alarm[index].start_time[i] = 0xffffffff;
			}
			alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 0;
			alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
	{
			return(ALARM_ACL_CLEARED);
	}	
	if(alarm_inst.cmpress_cycle_alarm[index].compress_state != ((g_sys.status.dout_bitmap >> DO_COMP2_BPOS)&0x0001))
	{
		if((alarm_inst.cmpress_cycle_alarm[index].compress_state == 0)&& //开压缩机判断
			(((g_sys.status.dout_bitmap >> DO_COMP2_BPOS)&0x0001) == 1))	
		{
			get_local_time(&now);
			alarm_inst.cmpress_cycle_alarm[index].start_time[0] = alarm_inst.cmpress_cycle_alarm[index].start_time[1];
			alarm_inst.cmpress_cycle_alarm[index].start_time[1] = alarm_inst.cmpress_cycle_alarm[index].start_time[2];
			alarm_inst.cmpress_cycle_alarm[index].start_time[2] = alarm_inst.cmpress_cycle_alarm[index].start_time[3];
			alarm_inst.cmpress_cycle_alarm[index].start_time[3] = alarm_inst.cmpress_cycle_alarm[index].start_time[4];
			alarm_inst.cmpress_cycle_alarm[index].start_time[4] = alarm_inst.cmpress_cycle_alarm[index].start_time[5];
			alarm_inst.cmpress_cycle_alarm[index].start_time[5] = alarm_inst.cmpress_cycle_alarm[index].start_time[6];
			alarm_inst.cmpress_cycle_alarm[index].start_time[6] = alarm_inst.cmpress_cycle_alarm[index].start_time[7];
			alarm_inst.cmpress_cycle_alarm[index].start_time[7] = alarm_inst.cmpress_cycle_alarm[index].start_time[8];
			alarm_inst.cmpress_cycle_alarm[index].start_time[8] = alarm_inst.cmpress_cycle_alarm[index].start_time[9];
			alarm_inst.cmpress_cycle_alarm[index].start_time[9] = now; 
			
			if((alarm_inst.cmpress_cycle_alarm[index].start_time[0] != 0xffffffff)&&
				((now-alarm_inst.cmpress_cycle_alarm[index].start_time[0] ) <= 3600))//1??????10????????
			{
				alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 1800;
				alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 1;
			}
				
		}

		alarm_inst.cmpress_cycle_alarm[index].compress_state = ((g_sys.status.dout_bitmap >> DO_COMP2_BPOS)&0x0001);
		
	}
	if(alarm_inst.cmpress_cycle_alarm[index].cycle_flag)
	{
		if(alarm_inst.cmpress_cycle_alarm[index].alarm_timer > 0 )
		{
			alarm_inst.cmpress_cycle_alarm[index].alarm_timer--;
			if((get_alarm_bitmap(ACL_HI_TEMP_RETURN)) || (get_alarm_bitmap(ACL_HI_TEMP_SUPPLY)))//高温报警，自动解锁
			{
					alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
					alarm_inst.cmpress_cycle_alarm[index].alarm_timer = 0;
				  return(0);
			}
			else
			{
			return(1);
			}
		}
		else
		{
			alarm_inst.cmpress_cycle_alarm[index].cycle_flag = 0;
			return(0);
		}
	}
	else
	{
		return(0);
	}
	

}

//ACL_COMPRESSOR_OT2	

static	uint16_t acl38(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if((devinfo_get_compressor_cnt() != 2)||(g_sys.config.general.cool_type == COOL_TYPE_MODULE_WATER))
		{
				return(ALARM_ACL_CLEARED);
		}		
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_COMP2_BPOS].low,g_sys.status.run_time[DO_COMP2_BPOS].high);
		max = g_sys.config.alarm[ACL_COMPRESSOR_OT2].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));	
}


//ACL_HUM_OCURRENT           

static	uint16_t acl39(alarm_acl_status_st* acl_ptr)
{
	uint16_t meter;
	uint16_t max;
	//参数确定
	//运行条件下才报警，没有运行保持

	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((g_sys.status.dout_bitmap & (0x0001 << DO_HUM_BPOS)) == 0)
	{
			return(ALARM_ACL_HOLD);
	}
	
	meter = g_sys.status.mbm.hum.hum_current;		
	max = HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap*150;//g_sys.config.alarm[ACL_HUM_OCURRENT].alarm_param;
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
	
	return(compare_calc( meter,0,max,THR_MAX_TYPE));		
}

//			ACL_HUM_HI_LEVEL			  ,
uint16_t acl40(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	data = g_sys.status.mbm.hum.water_level;
		
	return(io_calc( data,IO_CLOSE));
}
//			ACL_HUM_LO_LEVEL        ,
static	uint16_t acl41(alarm_acl_status_st* acl_ptr)
{
		extern local_reg_st		l_sys;
		uint16_t min, meter;
	
		
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			 return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.status.mbm.hum.water_level == 1)
		{
					return(ALARM_ACL_CLEARED);
		}
		if(((g_sys.status.dout_bitmap & (0x0001 << DO_HUM_BPOS)) == 0)||(l_sys.l_fsm_state[HUMIDIFIER_FSM_STATE]==HUM_FSM_STATE_WARM))
		{
				return(ALARM_ACL_HOLD);
		}
	
		min =g_sys.config.alarm[ACL_HUM_LO_LEVEL].alarm_param*HUM_CURRENT_UNIT*g_sys.config.humidifier.hum_real_cap;
		meter = g_sys.status.mbm.hum.hum_current;		
		
	//没有高水位触发的情况下，并且加湿电流过小
		if((compare_calc( meter,min,0,SML_MIN_TYPE) == 1)&&
			(io_calc( g_sys.status.mbm.hum.water_level,IO_CLOSE) == 0))
		{
				return(1);
		}
		else
		{
				return(0);
		}	
}
//			ACL_HUM_OT					    ,
static	uint16_t acl42(alarm_acl_status_st* acl_ptr)
{		
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			 return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_HUM_BPOS].low,g_sys.status.run_time[DO_HUM_BPOS].high);
		max = g_sys.config.alarm[ACL_HUM_OT].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));			
}
//ACL_HEATER_OD

 static uint16_t acl43(alarm_acl_status_st* acl_ptr)
{
		uint8_t data;
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			return(ALARM_ACL_CLEARED);
		}
		
		data = sys_get_di_sts(DI_HEATER_OD_BPOS);	
		//报警时记录几级加热
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = 0; 	

		if(g_sys.status.dout_bitmap &(0x0001<<DO_RH1_BPOS))
		{
				alarm_inst.alarm_sts[acl_ptr->id].alram_value |=0x0001;
		}

		if(g_sys.status.dout_bitmap &(0x0001<<DO_RH2_BPOS))
		{
				alarm_inst.alarm_sts[acl_ptr->id].alram_value |=0x0002;
		}

//		if(g_sys.status.dout_bitmap &(0x0001<<DO_RH3_BPOS))
//		{
//				alarm_inst.alarm_sts[acl_ptr->id].alram_value |=0x0004;
//		}
		return(io_calc( data,IO_CLOSE));
}

////加热器 运行超时
//			ACL_HEATER_OT1			    ,
static	uint16_t acl44(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_RH1_BPOS].low,g_sys.status.run_time[DO_RH1_BPOS].high);
		max = g_sys.config.alarm[ACL_HEATER_OT1].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));			
}
//			ACL_HEATER_OT2				  ,
static	uint16_t acl45(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_RH2_BPOS].low,g_sys.status.run_time[DO_RH2_BPOS].high);
		max = g_sys.config.alarm[ACL_HEATER_OT2].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));	
}
//			ACL_HEATER_OT3				  ,
//static	uint16_t acl46(alarm_acl_status_st* acl_ptr)
//{
//		uint32_t run_time;
//		int16_t max;
//		//参数确定
//	
//		run_time = g_sys.status.run_time[DO_RH3_BPOS].high;
//		run_time = run_time<<16|g_sys.status.run_time[DO_RH3_BPOS].low;	
//		run_time = run_time/OVER_TIME_ACCURACY;
//		
//	//	max = g_sys.config.alarm[ACL_HEATER_OT3].alarm_param;
//		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
//		return(compare_calc( run_time,0,max,THR_MAX_TYPE));	
//}

//    alarm_compare,//			ACL_POWER_LOSS					,//ABC三相电电压低于门限电压
static uint16_t acl46(alarm_acl_status_st* acl_ptr)
{
	  int8_t data;

		if(acl_clear(acl_ptr))
		{
			  sys_option_di_sts(DI_POWER_LOSS_BPOS,0);
				return(ALARM_ACL_CLEARED);
		}
		data = sys_get_di_sts(DI_POWER_LOSS_BPOS);		
		return(io_calc( data,IO_CLOSE));
}
//    alarm_io,//			ACL_POWER_EP						,	//reverse phase
static uint16_t acl47(alarm_acl_status_st* acl_ptr)
{
	int8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
	{
				return(ALARM_ACL_CLEARED);
	}
	if(((g_sys.config.dev_mask.dout & (0x01<<DO_PHASE_P_BPOS))==(0x01<<DO_PHASE_P_BPOS))||((g_sys.config.dev_mask.dout & (0x01<<DO_PHASE_N_BPOS))==(0x01<<DO_PHASE_N_BPOS)))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
	{
			data = g_sys.status.mbm.pwr.dev_sts&0x0001;
			return(io_calc( data,IO_CLOSE));
	}
	else
	{
			return(ALARM_ACL_CLEARED);
	}
}

//				ACL_POWER_HI_FD						,	//freqency deviation
static uint16_t acl48(alarm_acl_status_st* acl_ptr)
{
	
	int16_t meter;
	int16_t max ;
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
	{
				return(ALARM_ACL_CLEARED);
	}
	meter=g_sys.status.mbm.pwr.freq;

	max = g_sys.config.alarm[ACL_POWER_HI_FD].alarm_param;
	

	alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;

	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}
//ACL_POWER_LO_FD						,	//freqency deviation
static uint16_t acl49(alarm_acl_status_st* acl_ptr)
{
	
	int16_t meter;
	int16_t min ;
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
	{
				return(ALARM_ACL_CLEARED);
	}
	meter=g_sys.status.mbm.pwr.freq;

	min = g_sys.config.alarm[ACL_POWER_LO_FD].alarm_param;
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;

	return(compare_calc( meter,min,0,SML_MIN_TYPE));
	
}

//		alarm_compare,//			ACL_POWER_A_HIGH				,
static uint16_t acl50(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t max;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		meter = g_sys.status.mbm.pwr.pa_volt;
		
		max = g_sys.config.alarm[ACL_POWER_A_HIGH].alarm_param;
		
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
		
		return(compare_calc( meter,0,max,THR_MAX_TYPE));
}

//		alarm_compare,//			ACL_POWER_B_HIGH				,
static uint16_t acl51(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t max;
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				
				meter = g_sys.status.mbm.pwr.pb_volt;
						
				max = g_sys.config.alarm[ACL_POWER_A_HIGH].alarm_param ;
				
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
				return(compare_calc( meter,0,max,THR_MAX_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
}

//		alarm_compare,//			ACL_POWER_C_HIGH				,
static uint16_t acl52(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t max;
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				
				meter = g_sys.status.mbm.pwr.pc_volt;
				max = g_sys.config.alarm[ACL_POWER_A_HIGH].alarm_param;
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;	
				return(compare_calc( meter,0,max,THR_MAX_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
		
}

//		ACL_POWER_A_LOW					
static uint16_t acl53(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		meter = g_sys.status.mbm.pwr.pa_volt;
				
		min = g_sys.config.alarm[ACL_POWER_A_LOW].alarm_param; 
	
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
		return(compare_calc( meter,min,0,SML_MIN_TYPE));
}


//		alarm_compare,//			ACL_POWER_B_LOW					,
static uint16_t acl54(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				
				meter = g_sys.status.mbm.pwr.pb_volt;		
		    min = g_sys.config.alarm[ACL_POWER_A_LOW].alarm_param;
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
				return(compare_calc( meter,min,0,SML_MIN_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
		
}

//		alarm_compare,//			ACL_POWER_C_LOW					,
static uint16_t acl55(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;
	
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				meter = g_sys.status.mbm.pwr.pc_volt;	
				min = g_sys.config.alarm[ACL_POWER_A_LOW].alarm_param ;
				//min = min >>POWER_DOT_BIT;
				//min=((g_sys.config.alarm[ACL_POWER_C_LOW].alarm_param & POWER_DOT)/100)*min;
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
				return(compare_calc( meter,min,0,SML_MIN_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
		
}

//		alarm_io,//			ACL_POWER_A_OP					,	//open phase
static uint16_t acl56(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
					meter = g_sys.status.mbm.pwr.pa_volt;		
					min = g_sys.config.alarm[ACL_POWER_A_OP].alarm_param ;
					//	min = min >>POWER_DOT_BIT;
					//	min=((g_sys.config.alarm[ACL_POWER_A_OP].alarm_param & POWER_DOT)/100)*min;
					alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
					return(compare_calc( meter,min,0,SML_MIN_TYPE));
		}
		else
		{
					return(ALARM_ACL_CLEARED);
		}
		
}

//		alarm_io,//			ACL_POWER_B_OP					,	//open phase
static uint16_t acl57(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				meter = g_sys.status.mbm.pwr.pb_volt;
				min = g_sys.config.alarm[ACL_POWER_A_OP].alarm_param ;
				//min = min >>POWER_DOT_BIT;
				//min=((g_sys.config.alarm[ACL_POWER_B_OP].alarm_param & POWER_DOT)/100)*min;
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
				return(compare_calc( meter,min,0,SML_MIN_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
		
}


//		alarm_io,//			ACL_POWER_C_OP					,	//open phase
static uint16_t acl58(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t min;

		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(POWER_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		if(g_sys.config.ac_power_supply.power_mode == POWER_MODE_THREBLE)//三相电
		{
				
				meter = g_sys.status.mbm.pwr.pc_volt;	
				min = g_sys.config.alarm[ACL_POWER_A_OP].alarm_param ;
				//	min = min >>POWER_DOT_BIT;
				//min=((g_sys.config.alarm[ACL_POWER_C_OP].alarm_param & POWER_DOT)/100)*min;
				alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
				return(compare_calc( meter,min,0,SML_MIN_TYPE));
		}
		else
		{
				return(ALARM_ACL_CLEARED);
		}
		
}


////气流丢失 单独函数实现
//			ACL_AIR_LOSS				   
static	uint16_t acl59(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	if((alarm_inst.fan_timer < g_sys.config.alarm[ACL_AIR_LOSS].alarm_param)&&(alarm_inst.fan_timer != 0))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	data = sys_get_di_sts(DI_AIR_FLOW_BPOS);	
	return(io_calc( data,IO_CLOSE));
	
}
//过滤网超时运行
//ACL_FILTER_OT
static	uint16_t acl60(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			return(ALARM_ACL_CLEARED);
		}
		run_time = dev_runingtime(g_sys.status.run_time[DO_FILLTER_DUMMY_BPOS].low,g_sys.status.run_time[DO_FILLTER_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FILTER_OT].alarm_param;
		
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));	
}

////过滤网赌
//			ACL_FILTER_CLOG        ,
static	uint16_t acl61(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(acl_get_pwr_sts() == 0)
//	{
//			return(ALARM_ACL_CLEARED);
//	}
	data = sys_get_di_sts(DI_FILTER_BPOS);
	return(io_calc( data,IO_CLOSE));
}

//// 远程关机 报警
//			ACL_REMOTE_SHUT				 
static	uint16_t acl62(alarm_acl_status_st* acl_ptr)
{
	uint8_t data; 
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	data = sys_get_di_sts(DI_REMOTE_BPOS);
	return(io_calc( data,IO_CLOSE));
}
////地板积水
//			ACL_WATER_OVERFLOW		 ,	

static	uint16_t acl63(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	data = sys_get_di_sts(DI_WATER_LEAK_BPOS);
	return(io_calc( data,IO_CLOSE));
}
///
//群控失败告警
//ACL_GROUP_CONTROL_FAIL 
static	uint16_t acl64(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
//	if(acl_get_pwr_sts() == 0)
//	{
//			return(ALARM_ACL_CLEARED);
//	}
	if(g_sys.config.team.team_en == 1)
	{
		  data = sys_get_di_sts(DI_GROUP_DEFAULT_BPOS);	
			return(io_calc( data,IO_CLOSE));
			
	}
	else
	{
			return(ALARM_ACL_CLEARED);
	}
}

//变频器告警
//
static	uint16_t acl65(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_get_pwr_sts() == 0)
	{
		 return(ALARM_ACL_CLEARED);
	}
	data = sys_get_di_sts(DI_OSD_INVERTER_BPOS);
	return(io_calc( data,IO_CLOSE));
	
}

//注水电导率高告警
//ACL_WATER_ELEC_HI
static	uint16_t acl66(alarm_acl_status_st* acl_ptr)
{
		uint16_t max;
		uint16_t meter;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			 return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
	
		meter = g_sys.status.mbm.hum.conductivity;
		max = g_sys.config.alarm[ACL_WATER_ELEC_HI].alarm_param;
		
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
		return(compare_calc( meter,0,max,THR_MAX_TYPE));	
	
}
//注水电导率低告警
//ACL_WATER_ELEC_LO
static	uint16_t acl67(alarm_acl_status_st* acl_ptr)
{
		int16_t min;
		uint16_t meter;
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
			return(ALARM_ACL_CLEARED);
		}
		if(sys_get_mbm_online(HUM_MODULE_BPOS) == 0)
		{
				return(ALARM_ACL_CLEARED);
		}
		meter = g_sys.status.mbm.hum.conductivity;
		min = g_sys.config.alarm[ACL_WATER_ELEC_LO].alarm_param;
		
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = meter;
		return(compare_calc( meter,min,0,SML_MIN_TYPE));	
	
}

//烟雾报警
//DI_SMOKE_BPOS
static	uint16_t acl68(alarm_acl_status_st* acl_ptr)
{
	uint8_t data; 
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	data = sys_get_di_sts(DI_SMOKE_BPOS);
	return(io_calc( data,IO_CLOSE));
}

//			ACL_USER_DEFINE,    用户自定义告警
static	uint16_t acl69(alarm_acl_status_st* acl_ptr)
{
	uint8_t data; 
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	data = sys_get_di_sts(DI_USR_BPOS);
	return(io_calc( data,IO_CLOSE));
}
//备用电源切换告警
//ACL_BK_POWER,
static	uint16_t acl70(alarm_acl_status_st* acl_ptr)
{
	uint8_t data; 
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	data = sys_get_di_sts(DI_BK_POWER_BPOS);
	return(io_calc( data,IO_CLOSE));
}

//ACL_COIL_HI_TEM1                  
static	uint16_t acl71(alarm_acl_status_st* acl_ptr)
{
	int16_t meter;
	int16_t max;

	//机型判断
	if((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)||
			(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND))
	{
			return(ALARM_ACL_CLEARED);
	}
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	//关机后自动解除。
	if(sys_get_pwr_sts() == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
		//水阀关的情况下返回保持
	if(g_sys.status.aout[AO_WATER_VALVE] == 0)
	{
		return(ALARM_ACL_HOLD);
	}
// 冷启动延时
//	if(alarm_inst.power_on_timer < g_sys.config.alarm[ACL_COIL_HI_TEM1].alarm_param2)
//	{
//		acl_ptr->state = ALARM_FSM_INACTIVE;
//		return(ALARM_ACL_HOLD);
//	}
//参数确定
	if(((g_sys.config.dev_mask.ain) & (0x01<<DEV_AIN_CIOL_NTC1_BPOS)) == 0)
	{
			return(ALARM_ACL_CLEARED);
	}
	meter = (int16_t)(g_sys.status.ain[DEV_AIN_CIOL_NTC1_BPOS]);
	max = (int16_t)(g_sys.config.alarm[ACL_COIL_HI_TEM1].alarm_param);
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	
	return(compare_calc( meter,0,max,THR_MAX_TYPE));
	
}

//ACL_COIL_VALVE_DEFAULT  

static	uint16_t acl72(alarm_acl_status_st* acl_ptr)
{
		int16_t meter;
		int16_t max;
		//机型判断
		if((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)||
				(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	if((sys_get_pwr_sts() == 0)||((sys_get_remap_status(WORK_MODE_STS_REG_NO, COOLING_STS_BPOS) == 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO, DEMHUM_STS_BPOS) == 0)))
	{
			return(ALARM_ACL_CLEARED);
	}
		
		if(g_sys.status.aout[AO_WATER_VALVE] > g_sys.status.ain[AI_WATER_VALVE_FB])
		{
				meter = g_sys.status.aout[AO_WATER_VALVE] -g_sys.status.ain[AI_WATER_VALVE_FB]; 
		}
		else
		{
				meter =  g_sys.status.ain[AI_WATER_VALVE_FB] - g_sys.status.aout[AO_WATER_VALVE];
		}
		
	
		max = (int16_t)(g_sys.config.alarm[ACL_COIL_VALVE_DEFAULT].alarm_param);
		return(compare_calc( meter,0,max,THR_MAX_TYPE));
}

//ACL_COIL_BLOCKING  
static	uint16_t acl73(alarm_acl_status_st* acl_ptr)
{
	int16_t meter,data;
	int16_t max;

	//	//机型判断
	if((g_sys.config.general.cool_type == COOL_TYPE_MODULE_WIND)||
		(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND))
	{
			return(ALARM_ACL_CLEARED);
	} 
	
	if(acl_clear(acl_ptr))
	{
			return(ALARM_ACL_CLEARED);
	}
	
	if((sys_get_pwr_sts() == 0)||((sys_get_remap_status(WORK_MODE_STS_REG_NO, COOLING_STS_BPOS) == 0)&&(sys_get_remap_status(WORK_MODE_STS_REG_NO, DEMHUM_STS_BPOS) == 0)))
	{
			return(ALARM_ACL_CLEARED);
	}
	// 有断流开关的情况下，断流开关优先
	if(g_sys.config.dev_mask.din[0]&(0x01<<DI_BLOCKING_BPOS))
	{
			data = sys_get_di_sts(DI_BLOCKING_BPOS);	
		  return(io_calc( data,IO_CLOSE));
	}
	//配备回风传感器
	if(g_sys.status.sys_tem_hum.supply_air_hum == 0x7fff)
	{
			return(ALARM_ACL_CLEARED);
	}
	//水阀关的情况下返回保持
	if(g_sys.status.ain[AI_WATER_VALVE_FB] == 0)
	{
		return(ALARM_ACL_HOLD);
	}
	meter = (int16_t) g_sys.status.sys_tem_hum.supply_air_hum ;
	
	max = (int16_t)(g_sys.config.alarm[ACL_COIL_BLOCKING].alarm_param);
	
	alarm_inst.alarm_sts[acl_ptr->id].alram_value=meter;
	
	return(compare_calc( meter,0,max,THR_MAX_TYPE));	
}
//ACL_FAN_OVERLOAD6  
static	uint16_t acl74(alarm_acl_status_st* acl_ptr)
{	
		uint8_t data;
		//机型判断
		if(g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)
		{
				return(ALARM_ACL_CLEARED);
		} 
		//remove loc
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_HOLD);
		}
		data = sys_get_di_sts(DI_FAN06_OD_BPOS);
		return(io_calc( data,IO_CLOSE));
}
//ACL_FAN_OVERLOAD7                 
static	uint16_t acl75(alarm_acl_status_st* acl_ptr)
{
	uint8_t data;
		//机型判断
		if(g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)
		{
				return(ALARM_ACL_CLEARED);
		} 
		//remove loc
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_HOLD);
		}
		data = sys_get_di_sts(DI_FAN07_OD_BPOS);
		return(io_calc( data,IO_CLOSE));
	
}
//ACL_FAN_OVERLOAD8             
static	uint16_t acl76(alarm_acl_status_st* acl_ptr)
{
	
		uint8_t data;
		//机型判断
		if(g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)
		{
				return(ALARM_ACL_CLEARED);
		} 
		//remove loc
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(sys_get_pwr_sts() == 0)
		{
				return(ALARM_ACL_HOLD);
		}
		data = sys_get_di_sts(DI_FAN08_OD_BPOS);
		return(io_calc( data,IO_CLOSE));
}

//ACL_FAN_OT6 
static	uint16_t acl77(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//机型判断
		if((g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)||((g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN06_OD_BPOS)) == 0))
		{
				return(ALARM_ACL_CLEARED);
		} 
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN6_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN6_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT6].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}

//ACL_FAN_OT7  
static	uint16_t acl78(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//机型判断
		if((g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)||((g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN07_OD_BPOS)) == 0))
		{
				return(ALARM_ACL_CLEARED);
		} 
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN7_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN7_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT7].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}
//ACL_FAN_OT8  
static	uint16_t acl79(alarm_acl_status_st* acl_ptr)
{
		uint32_t run_time;
		int16_t max;
		//机型判断
		if((g_sys.config.general.cool_type != COOL_TYPE_COLUMN_WATER)||((g_sys.config.dev_mask.din[0]&(0x01<<DI_FAN08_OD_BPOS)) == 0))
		{
				return(ALARM_ACL_CLEARED);
		} 
		//参数确定
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
	
		run_time = dev_runingtime(g_sys.status.run_time[DO_FAN8_DUMMY_BPOS].low,g_sys.status.run_time[DO_FAN8_DUMMY_BPOS].high);
		max = g_sys.config.alarm[ACL_FAN_OT8].alarm_param;
		alarm_inst.alarm_sts[acl_ptr->id].alram_value = run_time;
		return(compare_calc( run_time,0,max,THR_MAX_TYPE));
}
//ACL_WATER_PUMP_HI_LIVEL
static	uint16_t acl80(alarm_acl_status_st* acl_ptr)
{
		uint8_t data;
	
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		//机型判断
		if((g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WATER)||
			(g_sys.config.general.cool_type == COOL_TYPE_COLUMN_WIND))
		{
				
				if(g_sys.status.dout_bitmap & (0x0001 << DO_PUMMP_BPOS))
				{
						data = sys_get_di_sts(DI_COND_HI_LEVEL_BPOS);	
						return(io_calc( data,IO_CLOSE));
				}
				else
				{
						return(ALARM_ACL_HOLD);
				}
			
		} 
		else
		{
					return(ALARM_ACL_CLEARED);
		}
		
		
}

//ACL_HUM_DEFAULT
static uint16_t acl81(alarm_acl_status_st* acl_ptr)
{
		uint8_t data;
	
		if(acl_clear(acl_ptr))
		{
			  sys_option_di_sts(DI_HUM_DEFAULT_BPOS,0);
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
				sys_option_di_sts(DI_HUM_DEFAULT_BPOS,0);
			 return(ALARM_ACL_CLEARED);
		}
		
		data = sys_get_di_sts(DI_HUM_DEFAULT_BPOS);	
		return(io_calc( data,IO_CLOSE));
}

//ACL_FLOW_DIFF
static  uint16_t acl82(alarm_acl_status_st* acl_ptr)
{
		extern local_reg_st l_sys;
	
		if(acl_clear(acl_ptr))
		{
				return(ALARM_ACL_CLEARED);
		}
		if(acl_get_pwr_sts() == 0)
		{
				sys_option_di_sts(DI_HUM_DEFAULT_BPOS,0);
			  return(ALARM_ACL_CLEARED);
		}
		// 风机模式判断
		if(g_sys.config.fan.mode != FAN_MODE_DIFF)
		{
				return(ALARM_ACL_CLEARED);
		}
		//fan_speed == max_speed
		if( l_sys.ao_list[AO_EC_FAN][BITMAP_REQ] >= g_sys.config.fan.max_speed )
		{
				if(g_sys.status.ain[AI_AIR_FLOW_DIFF] < (g_sys.config.fan.set_flow_diff - g_sys.config.fan.flow_diff_deadzone))
				{
							return(ALARM_ACL_TRIGGERED);
				}
				else
				{
						 return(ALARM_ACL_TRIGGERED);
				}
		}
		else
		{
				 return(ALARM_ACL_HOLD);
		}
}
