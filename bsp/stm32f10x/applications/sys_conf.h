#ifndef __SYS_CONF
#define __SYS_CONF

#include "sys_def.h"
#include "alarms.h"

enum
{
	DO_REV1_BPOS = 0,
	DO_PHASE_P_BPOS,
	DO_PHASE_N_BPOS,
	DO_FAN_BPOS,
	DO_EV_BPOS,
	DO_REV3_BPOS,
	DO_HUM_BPOS,
	DO_FILL_BPOS,
	DO_DRAIN_BPOS,
	DO_RH1_BPOS,
	DO_RH2_BPOS,
	DO_COMP1_BPOS,
	DO_COMP2_BPOS,
	DO_DEHUM1_BPOS,
	DO_HGBP_BPOS,
	DO_ALARM_BPOS,
	DO_FILLTER_DUMMY_BPOS,
	DO_FAN2_DUMMY_BPOS,
	DO_FAN3_DUMMY_BPOS,
	DO_FAN4_DUMMY_BPOS,
	DO_FAN5_DUMMY_BPOS,
	DO_FAN6_DUMMY_BPOS,
	DO_FAN7_DUMMY_BPOS,
	DO_FAN8_DUMMY_BPOS,
	DO_OUT_FAN_DUMMY_BPOS,
	DO_MAX_CNT,
};
#define DO_WATER_VALVE_DUMMY_BPOS DO_COMP1_BPOS
#define DO_PUMMP_BPOS DO_DEHUM1_BPOS

//20150906
//enum
//{
//		DO_HUM_BPOS=0,
//    DO_COMP1_BPOS,
//    DO_COMP2_BPOS,
//    DO_RH1_BPOS,
//    DO_RH2_BPOS,
//    DO_FILL_BPOS,
//    DO_DRAIN_BPOS,
//    DO_DEHUM1_BPOS,
//    DO_DEHUM2_BPOS,
//    DO_LIQ1_BPOS,
//		DO_FAN_BPOS,
//		DO_EV_BPOS,
//		DO_REV1_BPOS,
//    DO_LIQ2_BPOS,
//		DO_PHASE_BPOS,
//    DO_ALARM_BPOS,
//		DO_MAX_CNT
//};
//enum
//{
//		DO_FAN_BPOS=0,
//    DO_COMP1_BPOS,
//    DO_COMP2_BPOS,
//    DO_RH1_BPOS,
//    DO_RH2_BPOS,
//    DO_HUM_BPOS,
//    DO_FILL_BPOS,
//    DO_DRAIN_BPOS,
//    DO_DEHUM1_BPOS,
//    DO_DEHUM2_BPOS,
//    DO_RH3_BPOS,
//    DO_LIQ1_BPOS,
//    DO_LIQ2_BPOS,
//    DO_REV1_BPOS,
//    DO_PHASE_BPOS,
//    DO_ALARM_BPOS,
//		DO_MAX_CNT
//};

//冷冻水控制输出
//#define DO_CIOL1_BPOS    DO_MAX_CNT+5
//#define DO_CIOL2_BPOS    DO_MAX_CNT+6

//application delay
#define INIT_THREAD_THREAD_DELAY 1000
#define MODBUS_SLAVE_THREAD_DELAY 1005
#define MODBUS_MASTER_THREAD_DELAY 1010
#define TCOM_THREAD_DELAY 1015
#define TEAM_THREAD_DELAY 1020
#define MBM_FSM_THREAD_DELAY 1025
#define DI_THREAD_DELAY 1030
#define DAQ_THREAD_DELAY 1035
#define CORE_THREAD_DELAY 1040
#define SURV_THREAD_DELAY 1045
#define CPAD_THREAD_DELAY 1050
#define BKG_THREAD_DELAY 2000
#define TESTCASE_THREAD_DELAY 1060

enum
{
	AI_PRESSURE1 = 0,
	AI_PRESSURE2,
	AI_GENERAL1,
	AI_GENERAL2,
	AI_NTC1,
	AI_NTC2,
	AI_NTC3,
	AI_NTC4,
	AI_NTC5,
	AI_NTC6,
	AI_MAX_CNT
};
#define AI_WATER_VALVE_FB AI_PRESSURE1
#define AI_WATER_FLOW AI_PRESSURE2
#define AI_AIR_FLOW_DIFF AI_GENERAL2
enum
{
	AO_EC_FAN = 0,
	AO_EC_COMPRESSOR,
	AO_WATER_VALVE,
	AO_PREV_1,
	AO_PREV_2,
	AO_MAX_CNT,
};

///////////////////////////////////////////////////////////////
//system configuration
///////////////////////////////////////////////////////////////

typedef struct
{
	uint16_t id;
	uint16_t *reg_ptr;
	int16_t min;
	uint16_t max;
	uint16_t dft;
	uint8_t permission;
	uint8_t rw;
	uint8_t (*chk_ptr)(uint16_t pram);
} conf_reg_map_st;

typedef struct
{
	uint16_t id;
	uint16_t *reg_ptr;
	uint16_t dft;
	//uint8_t		rw;
} sts_reg_map_st;

//system component mask, if set 1 measn exist, otherwise absent
typedef struct
{
	uint16_t ain;
	uint16_t din[2];
	uint16_t aout;
	uint16_t dout;
	uint16_t mb_comp;
	uint16_t return_temp_mask;
	uint16_t supply_temp_mask;
	uint16_t cab_temp_mask;
	uint16_t din_bitmap_polarity[2];
} dev_mask_st;

typedef struct
{
	uint16_t voltage;
	uint16_t frequency;
	uint16_t power_mode; //1三相电还是0单相电
} power_supply_st;

//system_set
/*
@operating_mode:	system operating mode 
	0:	power-off
	1:	standalone
	2:	team
	3:	shut_down
@standalone_mode:	
	0:	automatic
	1:	manual
	2:	testing
@team_mode:	
	0:	mode1
	1:	mode2
	2:	mode3
	3:	mode4
@cool_type:	
	0:	wind cool
	1:	water cool
	2:	chilled water
@cpad_baudrate:	
	0:	9600
	1:	19200
	2:	38400
	3:	57600	
@surv_baudrate:	
	0:	9600
	1:	19200
@surv_addr:	1~32
*/
enum
{
	COOL_TYPE_MODULE_WIND = 0,
	COOL_TYPE_MODULE_WATER,
	COOL_TYPE_MODULE_MIX,
	COOL_TYPE_COLUMN_WIND,
	COOL_TYPE_COLUMN_WATER,

};
enum
{
	POWER_MODE_THREBLE = 0,
	POWER_MODE_SINGLE,
	POWER_MODE_NONE

};
#define ALARM_TOTAL_WORD 6
typedef struct
{
	uint16_t temp;
	uint16_t hum;
} temp_sensor_cali_st;
typedef struct
{
	uint16_t power_mode;							//power-off or power-on
	uint16_t standalone_timer;						//automatic, manual
	uint16_t cool_type;								//cooling type
	uint16_t cpad_baudrate;							//control pad communication baudrate
	uint16_t surv_baudrate;							//surveillance communication baudrate
	uint16_t surv_addr;								//surveillance communication address
	uint16_t diagnose_mode_en;						//diagnose mode enalbe
	uint16_t alarm_bypass_en;						//diagnose mode enalbe
	uint16_t testing_mode_en;						//test mode enalbe
	uint16_t power_mode_mb_en;						// modbuss power mode control enable
	uint16_t alarm_remove_bitmap[ALARM_TOTAL_WORD]; //reless alarm
	uint16_t ntc_cali[6];							// NTC cali
	temp_sensor_cali_st temp_sensor_cali[TEMP_HUM_SENSOR_NUM];
} conf_general_st;

//status_set
/*
@permission_level: control pad accesssible user permission level
	0:	lowest						
	1:	above lowest
	2:	below highest
	3:	highest
@running_mode: control pad accesssible user permission level
	0:	standalone_power-off						
	1:	standalone_on
	2:	team_poweroff
	3:	team_power_on

@running_mode: control pad accesssible user permission level
	bit0:	fatal error						
	bit1:	internal modbus bus communication error
	bit2:	survallance modbus bus communication error
	bit3:	can bus communication error
*/
enum
{
	SYS_ERR_INIT = 0,
	SYS_ERR_TEAM,
	SYS_ERR_MBM,
	SYS_ERR_MBS,
	SYS_ERR_CAN,
	SYS_ERR_CPAD,
};

enum
{
	GEN_STS_REG_NO = 0,
	MBM_COM_STS_REG_NO,
	WORK_MODE_STS_REG_NO,
	SENSOR_STS_REG_NO
};

enum
{
	PWR_STS_BPOS = 0,
	FAN_STS_BPOS,
	HEATING_STS_BPOS,
	COOLING_STS_BPOS,
	HUMING_STS_BPOS,
	DEMHUM_STS_BPOS,
	COOL_VALVE_BPOS,
	TEAM_STANDALONE_STS_BPOS = 8,
	TEAM_STS_BPOS,
	ALARM_BEEP_BPOS = 15
};

typedef struct
{
	uint16_t permission_level; //user authentication level
	uint16_t running_mode;	 //automatic, manual or testing
	uint16_t sys_error_bitmap; //system error status
} status_general_st;

enum
{
	P_ALOGORITHM = 0,
	PID_ALOGORITHM,
	FUZZY_ALOGORITHM
};

enum
{
	HUM_RELATIVE = 0,
	HUM_ABSOLUTE
};
// meter tem_hum
typedef struct
{
	uint16_t supply_air_temp;
	uint16_t return_air_temp;
	uint16_t remote_air_temp;
	uint16_t supply_air_hum;
	uint16_t return_air_hum;
	uint16_t remote_air_hum;
} sys_tem_hum_st;
//algorithm
typedef struct
{
	uint16_t temp_calc_mode;
	uint16_t temp_ctrl_mode;
	uint16_t hum_ctrl_mode;
	uint16_t ctrl_target_mode;
	uint16_t supply_air_temp;
	uint16_t return_air_temp;
	uint16_t remote_air_temp;
	uint16_t supply_air_hum;
	uint16_t return_air_hum;
	uint16_t remote_air_hum;
	uint16_t temp_precision;
	uint16_t hum_precision;
	uint16_t temp_deadband;
	uint16_t hum_deadband;
	uint16_t sample_interval;
	uint16_t temp_integ;
	uint16_t temp_diff;
	uint16_t pid_action_max;
	uint16_t temp_req_out_max;
} algorithm_st;

//compressor
typedef struct
{
	uint16_t type;
	uint16_t dehum_level;
	uint16_t startup_delay;
	uint16_t stop_delay;
	uint16_t min_runtime;
	uint16_t min_stoptime;
	uint16_t startup_lowpress_shield;
	uint16_t alter_mode;
	uint16_t alter_time;
	uint16_t start_interval;
	uint16_t liq_val_ahead_time;
	uint16_t liq_val_delay_time;
	uint16_t speed_upper_lim;
	uint16_t speed_lower_lim;
	uint16_t ec_comp_start_req;
} compressor_st;

//compressor
typedef struct
{
	uint16_t auto_mode_en;
	uint16_t max_opening;
	uint16_t min_opening;
	uint16_t set_opening;
	uint16_t start_req;
	uint16_t mod_priority;
	uint16_t action_delay;
	uint16_t temp_act_delay;
	uint16_t trace_mode;
	uint16_t act_threashold;
} watervalve_st;

//fan
typedef struct
{
	uint16_t type;
	uint16_t mode;
	uint16_t num;
	uint16_t startup_delay;
	uint16_t cold_start_delay;
	uint16_t stop_delay;
	uint16_t set_speed;
	uint16_t min_speed;
	uint16_t max_speed;
	uint16_t dehum_ratio;
	uint16_t dehum_min_speed;
	uint16_t set_flow_diff;
	uint16_t flow_diff_deadzone;
	uint16_t flow_diff_step;
	uint16_t flow_diff_delay;
	uint16_t target_suc_temp;
	uint16_t suc_temp_deadzone;
	uint16_t suc_temp_step;
	uint16_t suc_temp_delay;
	uint16_t noload_down;
} fan_st;

//heater
typedef struct
{
	uint16_t cascade_level;

} heater_st;

//humidifier
typedef struct
{
	uint16_t hum_type; //0额定加湿，1比例带加湿。
	uint16_t hum_cool_en;
	uint16_t hum_cool_k; //制冷加湿系数
	uint16_t hum_capacity;
	uint16_t hum_cap_type;
	uint16_t hum_real_cap;
	uint16_t flush_time;
	uint16_t flush_interval;
	uint16_t min_fill_interval; //注水间隔
	uint16_t drain_timer;		//排水时间
	uint16_t fill_cnt;
	uint16_t hum_time;
	uint16_t current_percentage;
	uint16_t no_heater_flg;
	uint16_t min_percentage;
} humidifier_st;

//humidifier

// dehum_dev
typedef struct
{
	uint16_t stop_dehum_temp;
} dehum_st;
//team set
typedef struct
{
	uint16_t team_en;		//team enable
	uint16_t mode;			//team mode 0,1,2,3
	uint16_t addr;			//team id
	uint16_t baudrate;		//team communication baudrate
	uint16_t total_num;		//units number in the team
	uint16_t backup_num;	//backup units
	uint16_t rotate_period; //upper byte:0:no rotate;1:daily;2:weekly;lower byte:week day(0:sunday,1:monday...)
	uint16_t rotate_time;   //upper byte:hour;lower byte:minite;
	uint16_t rotate_num;
	uint16_t rotate_manual;
	uint16_t cascade_enable;
} team_st;

///////////////////////////////////////////////////////////////
//system status
///////////////////////////////////////////////////////////////
//#define AI_INPUT_MAX 	64

//#define	AI_SUPPLY_AIR_TEMP_1_POS		0
//#define	AI_SUPPLY_AIR_TEMP_2_POS		1
//#define	AI_RETURN_AIR_TEMP_1_POS		2
//#define	AI_RETURN_AIR_TEMP_2_POS		3
//#define	AI_SUPPLY_AIR_HUM_1_POS			4
//#define	AI_SUPPLY_AIR_HUM_2_POS			5
//#define	AI_RETURN_AIR_HUM_1_POS			6
//#define	AI_RETURN_AIR_HUM_2_POS			7
//#define	AI_AD_1_POS									8
//#define	AI_AD_2_POS									9
//#define	AI_AD_3_POS									10
//#define	AI_AD_4_POS									11
//#define	AI_AD_5_POS									12
//#define	AI_AD_6_POS									13
//#define	AI_AD_7_POS									14
//#define	AI_AD_8_POS									15
//#define	AI_AD_9_POS									16
//#define	AI_AD_10_POS								17
//#define	AI_HUM_CURRENT_POS					18
//#define	AI_HUM_CONDUCT_POS					19
//#define	AI_POWER_V_PA_POS						20
//#define	AI_POWER_V_PB_POS						21
//#define	AI_POWER_V_PC_POS						22
//#define	AI_POWER_CURR_POS						23
//#define	AI_POWER_FREQ_POS						24

//analog_in
typedef struct
{
	uint16_t ai_data[AI_MAX_CNT];
	uint64_t ai_mask;
} ain_st;

#define DI_COMP_1_HI_TEMP_POS ((uint32_t)0x00000001 << 0)
#define DI_COMP_2_HI_TEMP_POS ((uint32_t)0x00000001 << 1)
#define DI_COMP_1_LOW_TEMP_POS ((uint32_t)0x00000001 << 2)
#define DI_COMP_2_LOW_TEMP_POS ((uint32_t)0x00000001 << 3)
#define DI_COMP_1_DISC_TEMP_POS ((uint32_t)0x00000001 << 4)
#define DI_COMP_2_DISC_TEMP_POS ((uint32_t)0x00000001 << 5)
#define DI_FAN_1_OVF_POS ((uint32_t)0x00000001 << 6)
#define DI_FAN_2_OVF_POS ((uint32_t)0x00000001 << 7)
#define DI_FAN_3_OVF_POS ((uint32_t)0x00000001 << 8)
#define DI_AIR_LOSS_POS ((uint32_t)0x00000001 << 9)
#define DI_FILTER_CLOG_POS ((uint32_t)0x00000001 << 10)
#define DI_WATER_OVER_FLOW_POS ((uint32_t)0x00000001 << 11)
#define DI_RMT_SHUT_POS ((uint32_t)0x00000001 << 12)
#define DI_HUM_WATER_LV ((uint32_t)0x00000001 << 13)
#define DI_RESERVE_2_POS ((uint32_t)0x00000001 << 14)
#define DI_RESERVE_3_POS ((uint32_t)0x00000001 << 15)

#define ST_PWR_PA_AB_POS ((uint32_t)0x00000001 << 16)
#define ST_PWR_PB_AB_POS ((uint32_t)0x00000001 << 17)
#define ST_PWR_PC_AB_POS ((uint32_t)0x00000001 << 18)
#define ST_HUM_WL_H_POS ((uint32_t)0x00000001 << 19)
#define ST_HUM_HC_H_POS ((uint32_t)0x00000001 << 20)
#define ST_HUM_WQ_L_POS ((uint32_t)0x00000001 << 21)

//Digtal input status
/*
bit map:
bit0: 	compressor 1 hi temp valve
bit1: 	compressor 2 hi temp valve
bit2: 	compressor 1 low temp valve
bit3: 	compressor 2 low temp valve
bit4: 	compressor 1 discharge temp valve
bit5:		compressor 2 discharge temp valve
bit6: 	fan 1 overload valve
bit7: 	fan 2 overload valve
bit8:		fan 3 overload valve
bit9:		air lost valve
bit10:	filter clog valve
bit11:	water overflow valve
bit12:	remote shut valve
bit13:	reserve1
bit14:	reserve2
bit15:	reserve3

bit16:	power phase A error 
bit17:	power phase B error 
bit18:	power phase C error 
bit19:	humidifier water level high
bit20:	humidifier heating current high
bit21:	humidifier conductivity low
*/

typedef struct
{
	uint32_t din_data;
	uint32_t din_mask;
} din_st;

///////////////////////////////////////////////////////////////
//system output status
///////////////////////////////////////////////////////////////

//analog_out
//this feature is not yet determined, reserve interface for future application
typedef struct
{
	int16_t ec_fan[3];
	int16_t vf_compressor[2];
	int16_t reserve_aout[2];
} aout_st;

//Digital output definition
typedef struct
{
	int16_t fan_out[MAX_FAN_NUM];
	int16_t compressor_out[MAX_COMPRESSOR_NUM];
	int16_t heater_out[MAX_HEATER_NUM];
	int16_t liq_val_bypass_out[MAX_COMPRESSOR_NUM];
	int16_t hot_gas_bypass_out[MAX_COMPRESSOR_NUM];
	int16_t humidifier_out;
	int16_t dehumidification_out;
	int16_t water_injection_out;
	int16_t common_alarm_out;
	int16_t scr_out;
	int16_t usr_out[DO_MAX_CNT];
} dout_st;

///////////////////////////////////////////////////////////////
//system log
///////////////////////////////////////////////////////////////
//alarm status
typedef struct
{
	int16_t alarm_id;
	time_t trigger_time;
} alarm_status_st;

//alarm history
typedef struct
{
	int16_t alarm_id;
	time_t trigger_time;
	time_t clear_time;
} alarm_history_st;

//alarm system runtime log, record components accumulative running time
/*
@comp_id:
	0:	compressor 1
	1:	compressor 2
	2:	fan 1
	3:	fan 2
	4:	fan 3
	5:	heater 1
	6:	heater 2
	7:	humidifier
@action:
	0:	deactivated
	1:	activated
@trigger_time:
	sys_time
*/

typedef struct
{
	int16_t comp1_runtime_day;
	int16_t comp1_runtime_min;
	int16_t comp2_runtime_day;
	int16_t comp2_runtime_min;
	int16_t fan1_runtime_day;
	int16_t fan1_runtime_min;
	int16_t fan2_runtime_day;
	int16_t fan2_runtime_min;
	int16_t fan3_runtime_day;
	int16_t fan3_runtime_min;
	int16_t heater1_runtime_day;
	int16_t heater1_runtime_min;
	int16_t heater2_runtime_day;
	int16_t heater2_runtime_min;
	int16_t humidifier_runtime_day;
	int16_t humidifier_runtime_min;
} sys_runtime_log_st;

//alarm system runtime log, record components change of output states
/*
@comp_id:
	0:	compressor 1
	1:	compressor 2
	2:	fan 1
	3:	fan 2
	4:	fan 3
	5:	heater 1
	6:	heater 2
	7:	humidifier
@action:
	0:	deactivated
	1:	activated
@trigger_time:
	sys_time
*/
typedef struct
{
	uint16_t comp_id;
	uint16_t action;
	time_t trigger_time;
	time_t clear_time;
} sys_status_log_st;

///////////////////////////////////////////////////////////////
//alarms definition
///////////////////////////////////////////////////////////////

//alarms: acl definition
/*
@id:			alarm id
@delay:		trigger&clear delay 
@timeout:	delay timeout count down
@trigger_time:	alarm trigger time
@enable mode:	alarm enable mode
`0x00:		enable
`0x01:		suspend
`0x02:		forbid
@enable mask:	alarm enable mask
'0x03:	all mode enable
'0x02:	enable or forbid 
'0x01:	enable or suspend
'0x00:	only enable
@alarm_param:	related paramter(eg. threshold)
@void (*alarm_proc): designated alarm routine check function
*/
//typedef struct alarm_acl_td
//{
//	uint16_t 					id;
//	uint16_t					delay;
//	uint16_t 					timeout;
//	uint16_t 					state;
//	time_t						trigger_time;
//	uint16_t					enable_mode;
//	uint16_t					enable_mask;
//	uint16_t 					alarm_param;
//	uint16_t (*alarm_proc)(struct alarm_acl_td* str_ptr);
//}alarm_acl_st;

typedef struct
{
	uint16_t id;
	uint16_t delay;
	uint16_t enable_mode;
	uint16_t alarm_param;
} alarm_acl_conf_st;

#define DO_UPDATE 0x5A
#define VAR_BUFLEN 50
typedef struct
{
	U16_UNION HardWare;	//硬件版本
	U16_UNION SoftWare;	//软件版本
	U8 DO[7];			   //数字输出
	U8 T_Address;		   //测试板通信波特率
	U16_UNION T_Baudrate;  //测试板通信波特率
	U8 PC_Address;		   //PC通信地址
	U16_UNION PC_Baudrate; //PC通信波特率
	U32 Test[2];
	U8 Buff[VAR_BUFLEN];
	U8 U1_state;
} Var_Info; //变量类数据

//typedef struct alarm_acl_td
//{
//	uint16_t 					id;
//	uint16_t 					state;
//	time_t						trigger_time;
//	uint16_t (*alarm_proc)(struct alarm_acl_td* str_ptr);
//}alarm_acl_status_st;

//team struct definition
//team status
/*
@temp:	sampled temperature
@hum:		sampled humidity
@alarm_sts:
	`bit0:cri_alarm:	critical alarm, which will cause online set to go offline
		0:	no alarm
		1:	alarm active
	`bit1:norm_alarm:	sampled temperature
		0:	no alarm
		1:	alarm active
	`bit2:hitemp_alarm:
		0:	no alarm
		1:	alarm active
	`bit7:alarm_flag:
		0:	no alarm
		1:	alarm active
@comp_num:
	`compressor_num:	number of compressors
	`heater_num:			number of heaters
@run_state:
	0:	standby
	1:	running
@offline_count:
	0:	standby
	1:	running
*/
typedef struct
{
	int16_t temp;
	int16_t hum;
	uint8_t alarm_sts;  //0:critical_alarm;1:normal_alarm;2:hitemp_alarm;7:alarm_flag
	uint8_t comp_num;   //[3:0]:compressor_num;[7:4]:heater_num;
	uint8_t run_states; //7:run status;[6:0]:offline time count
} team_status_st;

//team configuration
/*
@temp_set:							set temperature
@temp_precision_set:		set temperature precisiton
@temp_deadband_set:			set temperature deadband
@hum_set:								set humidity
@hum_precision_set:			set humidity precisiton
@hum_deadband_set:			set humidity deadband
@hitemp_alarm_set:			set high temperature alarm threshold
@lotemp_alarm_set:			set low temperature alarm threshold
@hihum_alarm_set:				set high humidity alarm threshold
@lohum_alarm_set:				set low humidity alarm threshold
*/
typedef struct
{
	int16_t temp_set;
	int16_t temp_precision_set;
	int16_t temp_deadband_set;
	int16_t hum_set;
	int16_t hum_precision_set;
	int16_t hum_deadband_set;
	int16_t hitemp_alarm_set;
	int16_t lotemp_alarm_set;
	int16_t hihum_alarm_set;
	int16_t lohum_alarm_set;
} team_param_st;

//team requirement
/*
@temp_req:				temperature requirement
@hum_req:					humidity requirement
@temp_ctrl_mode:	temperature control mode set
	0:	P algorithm
	1:	PID algorithm
	2:	reserve
@hum_ctrl_mode:		humidity control mode set
	0:	P algorithm
	1:	reserve
@team_mode:			team operating mode set
	0:	standalone mode
	1:	average req mode
	2:	uni-direction mode
@run_enable:			run permission 
	0:	disable
	1:	enable
*/
typedef struct
{
	int16_t temp_req;
	int16_t hum_req;
	int8_t temp_ctrl_mode;
	int8_t hum_ctrl_mode;
	int8_t team_mode;
	int8_t run_enable;
} team_req_st;

//system memory map
//system memory configuration map
typedef struct sys_conf_map
{
	int16_t id;
	void *str_ptr;
	int16_t length;
} sys_conf_map_st;

typedef struct sys_status_map
{
	int16_t id;
	int16_t *addr;
	int16_t length;
} sys_status_map_st;
typedef struct
{
	uint16_t ext_fan_cnt;
	uint16_t ext_fan_psens_l_lim;
	uint16_t ext_fan_psens_h_lim;
	uint16_t ext_fan_prop_start;
	uint16_t ext_fan_prop_band;
	uint16_t ext_fan_prop_hyst;
	uint16_t ext_fan_prop_plat;
	uint16_t ext_fan_min_speed;
	uint16_t ext_fan_max_speed;
	uint16_t ext_fan1_set_speed;
	uint16_t ext_fan2_set_speed;
} ext_fan_st;

typedef struct
{
	uint16_t on_req;
	uint16_t hysterisis;
	uint16_t max_on_time;
	uint16_t min_off_time;
} hgbp_st;

typedef struct
{
	conf_general_st general;
	dev_mask_st dev_mask;
	algorithm_st algorithm;
	compressor_st compressor;
	watervalve_st water_valve;
	fan_st fan;
	heater_st heater;
	humidifier_st humidifier;
	dehum_st dehumer;
	team_st team;
	power_supply_st ac_power_supply;
	alarm_acl_conf_st alarm[ACL_TOTAL_NUM];
	ext_fan_st ext_fan_inst;
	hgbp_st hgbp;
} config_st;

typedef struct
{
	uint16_t dev_sts;
	uint16_t conductivity;
	uint16_t hum_current;
	uint16_t water_level;
} mbm_hum_st;

typedef struct
{
	uint16_t dev_sts;
	uint16_t pa_volt;
	uint16_t pb_volt;
	uint16_t pc_volt;
	uint16_t freq;
	uint16_t pe_bitmap;
} mbm_pwr_st;

typedef struct
{
	uint16_t dev_sts;
	uint16_t temp;
	uint16_t hum;
} mbm_tnh_st;
//modbus master data structure
typedef struct
{
	uint16_t dev_sts;
	uint16_t di_bit_map;
	uint16_t press;
	uint16_t temp;
	uint16_t fan_speed;
	uint16_t do_bit_map_r;
	uint16_t compress_speed_r;
	uint16_t do_bit_map_w;
	uint16_t compress_speed_w;
} mbm_outsidec_st;
//modbus master data structure
typedef struct
{
	uint8_t timer;
	mbm_hum_st hum; //4
	mbm_pwr_st pwr; //5
	mbm_outsidec_st outside_control;
	mbm_tnh_st tnh[TEMP_HUM_SENSOR_NUM]; //16
} mbm_sts_st;

//system information
typedef struct
{
	uint16_t status_reg_num;
	uint16_t config_reg_num;
	uint16_t software_ver;
	uint16_t hardware_ver;
	uint16_t serial_no[4];
	uint16_t man_date[2];
} sys_info_st;

typedef struct
{
	uint16_t low;
	uint16_t high;
} run_time_st;

typedef struct
{
	uint16_t critical_cnt;
	uint16_t major_cnt;
	uint16_t mioor_cnt;
	uint16_t total_cnt;
} alarm_state_cnt_st;

typedef struct
{
	uint16_t work_mode;
	uint16_t limit_time;
	uint16_t runing_time;
	uint16_t runing_hour;
	uint8_t pass_word[3];
} work_mode_st;

typedef enum
{
	RETURN_AIR_PLUSS_MODE = 0,
	SET_FAN_SPEED_MODE,
} return_air_mode_st;
typedef struct
{
	uint16_t timer;
	return_air_mode_st return_air_work_mode;
} return_air_sta_st;
typedef struct
{
	sys_info_st sys_info;
	status_general_st general;   //3
	mbm_sts_st mbm;				 //25
	uint16_t ain[AI_MAX_CNT];	//10
	uint16_t aout[AO_MAX_CNT];   //6
	uint16_t din_bitmap[2];		 //1
	uint16_t dout_bitmap;		 //1
	uint16_t status_remap[4];	//4
	uint16_t alarm_bitmap[6];	//4
	uint16_t flash_program_flag; //1
	run_time_st run_time[DO_MAX_CNT];
	alarm_state_cnt_st alarm_status_cnt;
	sys_tem_hum_st sys_tem_hum;
	work_mode_st sys_work_mode;
	uint16_t flow_diff_timer;
	return_air_sta_st return_air_status;
} status_st;

typedef struct
{
	config_st config;
	status_st status;
} sys_reg_st;

//yxq

////modebusó2?t???????ù
//typedef enum
//{
//	RETURN_TEM_HUM_SENSOR1_BPOS=0,
//	RETURN_TEM_HUM_SENSOR2_BPOS,
//	RETURN_TEM_HUM_SENSOR3_BPOS,
//	RETURN_TEM_HUM_SENSOR4_BPOS,
//	SUPPLY_TEM_HUM_SENSOR1_BPOS,
//	SUPPLY_TEM_HUM_SENSOR2_BPOS,
//	TEM_HUM_SENSOR_RESERVE1_BPOS,
//	TEM_HUM_SENSOR_RESERVE2_BPOS,
//	//
//	HUM_MODULE_BPOS,
//	POWER_MODULE_BPOS,
//	MBM_DEV_OSC_BPOS,//outside control board
//
//}DEV_MASK_MB_BPOS;

typedef enum
{
	RETURN_TEM_HUM_SENSOR_BPOS = 0,
	SUPPLY_TEM_HUM_SENSOR_BPOS,
	TEM_HUM_SENSOR3_BPOS,
	TEM_HUM_SENSOR4_BPOS,
	TEM_HUM_SENSOR5_BPOS,
	TEM_HUM_SENSOR6_BPOS,
	TEM_HUM_SENSOR7_BPOS,
	TEM_HUM_SENSOR8_BPOS,
	//
	HUM_MODULE_BPOS,
	POWER_MODULE_BPOS,
	MBM_DEV_OSC_BPOS, //outside control board

} DEV_MASK_MB_BPOS;
//?￡?aê?è?ó2?t???????ù
typedef enum
{
	DEV_AIN_PRESSURE1_BPOS = 0,
	DEV_AIN_PRESSURE2_BPOS,
	DEV_AIN_RESERVE1_BPOS,
	DEV_AIN_RESERVE2_BPOS,
	DEV_AIN_RETUREN_NTC1_BPOS,
	DEV_AIN_RETUREN_NTC2_BPOS,
	DEV_AIN_SUPPLY_NTC1_BPOS,
	DEV_AIN_SUPPLY_NTC2_BPOS,
	DEV_AIN_CIOL_NTC1_BPOS,
	DEV_AIN_CIOL_NTC2_BPOS,

} DEV_AIN_BPOS;

#endif //	__SYS_CONF
