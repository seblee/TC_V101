#ifndef _DAQ_H
#define _DAQ_H

#include "sys_conf.h"
enum
{
	TARGET_MODE_RETURN=0,
	TARGET_MODE_SUPPLY,\
	TARGET_MODE_REMOTE,
};
enum
{
		MAX_TEMP_MODE =0 ,
		AVERAGE_TEMP_MODE,
};
int16_t get_current_temp(uint8_t type);

uint16_t get_current_hum(uint8_t type);


#endif
