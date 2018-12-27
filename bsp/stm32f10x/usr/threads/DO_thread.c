#include <rtthread.h>
#include <components.h>

#include "sys_conf.h"
#include "spi_bsp.h"
#include "global_var.h"
#include "Kits_Memory.h"

//#define TEST 1

void DO_thread_entry(void *parameter)
{
	unsigned char Tmpbuf[6] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};

	drv_spi_init();
	g_Var_inst.DO[6] |= DO_UPDATE;
	rt_kprintf("DO_thread\n");
	while (1)
	{
#ifdef TEST
		Write_74HC595(NUM_74HC595, Tmpbuf); //写74HC595
#else
		if (g_Var_inst.DO[6] == DO_UPDATE) //刷新继电器输出
		{
			g_Var_inst.DO[6] &= ~DO_UPDATE;
			// 复制数据
			ReverseCopy(Tmpbuf, g_Var_inst.DO, 6);
			rt_kprintf("-----------0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x02%x\n", Tmpbuf[0], Tmpbuf[1], Tmpbuf[2], Tmpbuf[3], Tmpbuf[4], Tmpbuf[5]);
			Write_74HC595(NUM_74HC595, Tmpbuf); //写74HC595
		}
#endif
		rt_thread_delay(500);
	}
}
