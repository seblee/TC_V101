/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
#include <rtthread.h>
#include <stm32f10x.h>
#include "led_bsp.h"

#define led1_rcc                    RCC_APB2Periph_GPIOC
#define led1_gpio                   GPIOC
#define led1_pin                    (GPIO_Pin_13)

/**
  * @brief  LED initialization
  * @param  none
  * @retval none
  */
void drv_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(led1_rcc,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = led1_pin;
    GPIO_Init(led1_gpio, &GPIO_InitStructure);
	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	

	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/**
  * @brief  turn LED on
  * @param  none
  * @retval none
  */
void led_on(uint32_t n)
{
    switch (n)
    {
    case 0:
        GPIO_SetBits(led1_gpio, led1_pin);
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        break;
    default:
        break;
    }
}

/**
  * @brief  turn LED off
  * @param  none
  * @retval none
  */
void led_off(uint32_t n)
{
    switch (n)
    {
    case 0:
        GPIO_ResetBits(led1_gpio, led1_pin);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        break;
    default:
        break;
    }
}

