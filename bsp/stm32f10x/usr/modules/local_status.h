#ifndef __LOCAL_REG_H__
#define __LOCAL_REG_H__

#include "stdint.h"
#include "sys_conf.h"

enum
{
		RESET_REQ = 0,
		LOCAL_REQ,
		TEAM_REQ,
		TARGET_REQ,
		MAX_REQ,
		REQ_MAX_CNT,
};

enum
{
		T_REQ = 0,
		H_REQ,
		REQ_MAX_LEVEL,
};

//global FSM states definition
enum
{
		T_FSM_TOP_ID = 0,
		T_FSM_STANALONE_ID,
		T_FSM_TEAM_ID,
		H_FSM_MAX_ID_CNT
};

//global top-level FSM states definition
enum
{
		T_FSM_STATE_IDLE=0,
		T_FSM_STATE_STANDALONE,
		T_FSM_STATE_TEAM,
		T_FSM_MAX_CNT,
};

enum
{
		BITMAP_REQ=0,
		BITMAP_ALARM,
		BITMAP_MANUAL,
		BITMAP_FINAL,
		BITMAP_MASK,
		BITMAP_MAX_CNT,
};

enum
{
		FAN_FSM_STATE=0,
		COMPRESS_SIG_FSM_STATE,
		COMPRESS1_FSM_STATE,
		COMPRESS2_FSM_STATE,
		HEATER_FSM_STATE,
		DEHUMER_FSM_STATE,
		HUMIDIFIER_FSM_STATE,
    WATERVALVE_FSM_STATE,
		L_FSM_STATE_MAX_NUM,
};

enum
{
		T_FSM_SIG_IDLE=0,
		T_FSM_SIG_STANDALONE,
		T_FSM_SIG_TEAM,
		T_FSM_SIG_SHUT
};

/*
@signal:	system operating mode 
	1:	power on signal
	2:	power down signal
	3:	sys on signal
	4:	sys down signal
*/

typedef struct
{
		int16_t p_saved;
		int16_t i_saved;
		int16_t req_saved;
}pid_reg_st;

typedef struct
{
    int16_t set_point;   //Desired Value
    double  proportion; //Proportional Const
    double  integral;   //Integral Const
    double  derivative; //Derivative Const
    int16_t last_error;  //Error[-1]
    int16_t prev_error;  //Error[-2]
}pid_param_st;

typedef struct
{
		pid_reg_st temp;
		pid_reg_st hum;
}pid_st;

typedef struct 
{
		int16_t 	require[REQ_MAX_CNT][REQ_MAX_LEVEL];		
		uint16_t 	bitmap[BITMAP_MAX_CNT];
		int16_t 	ao_list[AO_MAX_CNT][BITMAP_MAX_CNT];
		uint16_t 	comp_timeout[DO_MAX_CNT];
		uint16_t	t_fsm_state;
		uint16_t	t_fsm_signal;
		int16_t   ec_fan_diff_reg;
		int16_t   ec_fan_suc_temp;
		uint16_t  authen_cd;
		uint16_t 	comp_startup_interval;
		uint16_t	debug_flag;
		uint16_t	debug_tiemout;
		uint16_t 	l_fsm_state[L_FSM_STATE_MAX_NUM];
		pid_st		pid;
    uint16_t  watervalve_warmup_delay;
}local_reg_st;


#endif //__LOCAL_REG_H__
