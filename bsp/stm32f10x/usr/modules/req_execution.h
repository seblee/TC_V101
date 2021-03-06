#ifndef __REQ_EXE_H__
#define __REQ_EXE_H__
#include "stdint.h"
#define HUM_CURRENT_UNIT                  1.19
enum
{
		HUM_FSM_STATE_IDLE = 0,
		HUM_FSM_STATE_CHECK,
		HUM_FSM_STATE_WARM,
		HUM_FSM_STATE_DRAIN,
		HUM_FSM_STATE_HUM,
		HUM_FSM_STATE_FILL,
		HUM_FSM_STATE_FLUSH,
	
};

enum
{
		COMPRESSOR_FSM_STATE_IDLE=0,
		COMPRESSOR_FSM_STATE_INIT,
		COMPRESSOR_FSM_STATE_STARTUP,
		COMPRESSOR_FSM_STATE_NORMAL,
		COMPRESSOR_FSM_STATE_SHUTING,
		COMPRESSOR_FSM_STATE_STOP,
};

enum
{
    WATERVALVE_FSM_STATE_STOP = 0,
    WATERVALVE_FSM_STATE_STARTUP,
    WATERVALVE_FSM_STATE_WARMUP,
    WATERVALVE_FSM_STATE_WARMUP_SHUTING,
    WATERVALVE_FSM_STATE_NORMAL,
    WATERVALVE_FSM_STATE_SHUTING,
};
enum
{
		FAN_MODE_FIX=0,
		FAN_MODE_FLEX,
		FAN_MODE_DIFF
};
void hum_capacity_calc(void);
void req_execution(int16_t target_req_temp,int16_t target_req_hum);
void req_bitmap_op(uint8_t component_bpos, uint8_t action);
#endif //__REQ_EXE_H__

