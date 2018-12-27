/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2013-07-12     aozima       update for auto initial.
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f10x.h"
#include "stm32f10x_fsmc.h"
#include "board.h"
#include "usart_bsp.h"
#include "adc_bsp.h"
#include "i2c_bsp.h"
#include "dio_bsp.h"
#include "led_bsp.h"
#include "pwm_bsp.h"
#include "rtc_bsp.h"
#include "can_bsp.h"
#include "spi_bsp.h"

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
   NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);//IAP BOOTUP ADDRESS
	//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);//standalone bootup address
#endif
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init(void)
{
    /* NVIC Configuration */
    NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );

    drv_usart_init();
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);	
	
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

void hw_drivers_init(void)
{
//	drv_adc_dma_init();
	
//	
	
// 	drv_dio_init();
	
//	drv_pwm_init();
	
//	drv_i2c_init();
	
//	drv_rtc_init();	

//	drv_can_init();

//	drv_spi_init();
	
//	drv_led_init();
}
/*@}*/
