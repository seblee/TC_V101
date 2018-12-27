/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include <math.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif
#include "stm32f10x_flash.h"
#include "thread_entries.h"
#include "global_var.h"
//#include "event_record.h"
//#include "password.h"
#define FLASH_APP_FLAG_ADDR		0x08002f80   
#define FLASH_APP_FLAG_WORD		0xa5a5

//enum
//{
//		INIT_THREAD_THREAD_PRIO=7,
//		MODBUS_SLAVE_THREAD_PRIO,
//		MONITOR_SLAVE_THREAD_PRIO,
//		MODBUS_MASTER_THREAD_PRIO,
//		TCOM_THREAD_PRIO,	
//		TEAM_THREAD_PRIO,
//		MBM_FSM_THREAD_PRIO,					
//		DI_THREAD_PRIO,							
//		DAQ_THREAD_PRIO,							
//		CORE_THREAD_PRIO,						
//		SURV_THREAD_PRIO,						
//		CPAD_THREAD_PRIO,					
//		BKG_THREAD_PRIO,
//		TESTCASE_THREAD_PRIO,		
//		USR_MAX_PRIO
//};
//
//ALIGN(RT_ALIGN_SIZE)
//static rt_uint8_t modbus_master_stack[ 512 ];
//static rt_uint8_t modbus_slave_stack[ 512 ];
//static rt_uint8_t monitor_slave_stack[ 512 ];
//static rt_uint8_t team_stack[ 512 ];
//static rt_uint8_t mbm_fsm_stack[ 512 ];
//static rt_uint8_t di_stack[ 256 ];
//static rt_uint8_t daq_stack[ 1024 ];
//static rt_uint8_t core_stack[ 1024 ];
//static rt_uint8_t cpad_stack[ 3072 ];
//static rt_uint8_t tcom_stack[ 512 ];
//static rt_uint8_t bkg_stack[ 512 ];
//static rt_uint8_t testcase_stack[ 2048 ];
//
//static struct rt_thread modbus_master_thread;
//static struct rt_thread modbus_slave_thread;
//static struct rt_thread monitor_slave_thread;
//static struct rt_thread tcom_thread;
//static struct rt_thread team_thread;
//static struct rt_thread mbm_fsm_thread;
//static struct rt_thread di_thread;
//static struct rt_thread daq_thread;
//static struct rt_thread core_thread;
//static struct rt_thread cpad_thread;
//static struct rt_thread bkg_thread;
//static struct rt_thread testcase_thread;

enum
{
		INIT_THREAD_THREAD_PRIO=7,
		COMM_PC_THREAD_PRIO,
		COMM_T_THREAD_PRIO,	
		DO_THREAD_PRIO,					
		BKG_THREAD_PRIO,	
		TESTCASE_THREAD_PRIO,
		USR_MAX_PRIO
};

ALIGN(RT_ALIGN_SIZE)

static rt_uint8_t Comm_PC_stack[ 512 ];
static rt_uint8_t Comm_T_stack[ 512 ];
static rt_uint8_t DO_stack[ 512 ];
static rt_uint8_t Bkg_stack[ 512 ];

static struct rt_thread Comm_PC_thread;
static struct rt_thread Comm_T_thread;
static struct rt_thread DO_thread;
static struct rt_thread Bkg_thread;

//static struct rt_thread do_thread;


void set_boot_flag(void);

void rt_init_thread_entry(void* parameter)
{
#ifdef	RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

#ifdef	RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */
//		hw_drivers_init(); //
		sys_global_var_init();
//		sys_local_var_init();		
//		drv_can_init();
//		init_work_mode();
//	  init_evnet_log();
//	  init_alarm_log();
	

		if((*(__IO uint32_t*)FLASH_APP_FLAG_ADDR) != FLASH_APP_FLAG_WORD)
		{
			set_boot_flag();
		}

}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

	//
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, INIT_THREAD_THREAD_PRIO, 20);

    if (init_thread != RT_NULL)
	{
        rt_thread_startup(init_thread);	
	}

 	result = rt_thread_init(&Comm_PC_thread,//PC端通信进程
                            "Comm_PC",
                            Comm_PC_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&Comm_PC_stack[0],
                            sizeof(Comm_PC_stack),
                            COMM_PC_THREAD_PRIO,
                            5);
	if (result == RT_EOK)
    {
        rt_thread_startup(&Comm_PC_thread);
    }	
	result = rt_thread_init(&Comm_T_thread,//测试板通信进程
                            "Comm_T",
                            Comm_T_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&Comm_T_stack[0],
                            sizeof(Comm_T_stack),
                            COMM_T_THREAD_PRIO,
                            5);
	if (result == RT_EOK)
    {
        rt_thread_startup(&Comm_T_thread);
    }	
	result = rt_thread_init(&DO_thread,//继电器数字输出进程
                            "DO",
                            DO_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&DO_stack[0],
                            sizeof(DO_stack),
                            DO_THREAD_PRIO,
                            5);

    if (result == RT_EOK)
    {
        rt_thread_startup(&DO_thread);
    }	

    result = rt_thread_init(&Bkg_thread,//背景进程，LED指示灯
                            "background",
                            bkg_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&Bkg_stack[0],
                            sizeof(Bkg_stack),
                            BKG_THREAD_PRIO,
                            5);
    if (result == RT_EOK)
    {
        result = rt_thread_startup(&Bkg_thread);
    }	
						
    return 0;
}

void set_boot_flag(void)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	FLASH_ErasePage(FLASH_APP_FLAG_ADDR);
	FLASH_ProgramWord(FLASH_APP_FLAG_ADDR, FLASH_APP_FLAG_WORD);
	FLASH_Lock();
}

void clear_boot_flag(void)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	FLASH_ErasePage(FLASH_APP_FLAG_ADDR);
	FLASH_Lock();
}

void sys_reboot(void)
{
	NVIC_SystemReset();
}

void reload_sys(void)
{
	clear_boot_flag();
	sys_reboot();
}

FINSH_FUNCTION_EXPORT(set_boot_flag, set boot flag in flashrom.);
FINSH_FUNCTION_EXPORT(clear_boot_flag, clear boot flag in flashrom.);
FINSH_FUNCTION_EXPORT(sys_reboot, software reset.);
FINSH_FUNCTION_EXPORT(reload_sys, reset system and clear boot flag.);

/*@}*/



