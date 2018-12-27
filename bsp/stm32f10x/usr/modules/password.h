#ifndef _PASSWORD_H
#define _PASSWORD_H

#include "sys_conf.h"
enum
{
	WORK_MODE_FSM_OPEN = 0xE1,
	WORK_MODE_FSM_MANAGE = 0xD2,
	WORK_MODE_FSM_LOCK = 0xC3,
	WORK_MODE_FSM_LIMIT = 0xB4,
};

void init_work_mode(void);

void work_mode_manage(void);

uint8_t passward_compare(uint8_t *st1,uint8_t *st2, uint8_t len);
uint8_t cpad_work_mode(uint8_t work_mode,uint16_t day_limit);

uint8_t  write_passward (uint8_t*password, uint8_t work_mode,uint16 limit_time);

#endif

