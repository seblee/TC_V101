#include <rtthread.h>
#include "sys_conf.h"
#include "team.h"
#include "can_bsp.h"

 	

		

void team_thread_entry(void* parameter)
{
		rt_thread_delay(TEAM_THREAD_DELAY);
		
		while(1)
		{
//				team_fsm_trans();		
				rt_thread_delay(2000);
		}
}

