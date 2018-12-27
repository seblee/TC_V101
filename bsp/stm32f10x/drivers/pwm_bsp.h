#ifndef __PWN_H__
#define __PWN_H__
#include "stdint.h"

void drv_pwm_init(void);
uint16_t pwm_set_ao(uint8_t channel, uint16_t ao_data);
#endif //__PWN_H__
