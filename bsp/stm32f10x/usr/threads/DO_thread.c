#include <rtthread.h>
#include <components.h>

#include "sys_conf.h"
#include "spi_bsp.h"
#include "global_var.h"
#include "Kits_Memory.h"

//#define TEST 1

struct rx_msg
{
	rt_device_t dev;
	rt_size_t size;
};

/* 用于接收消息的消息队列*/
static rt_mq_t rx_mq = RT_NULL;
/* 接收线程的接收缓冲区*/
static unsigned char uart_rx_buffer[64];
static unsigned char count_rx_buffer[1024];
/* 数据到达回调函数*/
rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
	struct rx_msg msg;
	msg.dev = dev;
	msg.size = size;
	/* 发送消息到消息队列中*/
	rt_mq_send(rx_mq, &msg, sizeof(struct rx_msg));
	return RT_EOK;
}
void process_data_of_u1(unsigned char *data, rt_uint8_t len);
rt_uint8_t Serial_State_U1_OF_M1(rt_uint8_t *state);
void DO_thread_entry(void *parameter)
{
	unsigned char Tmpbuf[6] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
	struct rx_msg msg;
	rt_device_t device;
	rt_err_t result = RT_EOK;
	rt_uint32_t count = 0;

	rx_mq = rt_mq_create("rxmq", sizeof(struct rx_msg), 5, RT_IPC_FLAG_FIFO);
	if(rx_mq == RT_NULL)
    {
        
    }
    device = rt_device_find("uart4");
	if (device != RT_NULL)
	{
		/* 设置回调函数及打开设备*/
		rt_device_set_rx_indicate(device, uart_input);
		rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	}

	drv_spi_init();
	g_Var_inst.DO[6] |= DO_UPDATE;
	rt_kprintf("DO_thread\n");
	while (1)
	{
		/* 从消息队列中读取消息*/
		result = rt_mq_recv(rx_mq, &msg, sizeof(struct rx_msg), 500);
		if (result == -RT_ETIMEOUT)
		{
			if (count > 0)
			{
				process_data_of_u1(count_rx_buffer, count);
				rt_kprintf("count %d\n", count);
				count = 0;
				rt_kprintf("rec_state %d\n", Serial_State_U1_OF_M1(RT_NULL));
			}
#ifdef TEST
			Write_74HC595(NUM_74HC595, Tmpbuf); //写74HC595
#else
			if (g_Var_inst.DO[6] == DO_UPDATE) //刷新继电器输出
			{
				g_Var_inst.DO[6] &= ~DO_UPDATE;
				// 复制数据
				ReverseCopy(Tmpbuf, g_Var_inst.DO, 6);
				rt_kprintf("-----------0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x02%x\n", Tmpbuf[0], Tmpbuf[1], Tmpbuf[2], Tmpbuf[3], Tmpbuf[4], Tmpbuf[5]);
				if ((Tmpbuf[0] == 0) && (Tmpbuf[1] == 0) && (Tmpbuf[2] == 0) && (Tmpbuf[3] == 0) && (Tmpbuf[4] == 0) && (Tmpbuf[5] == 0))
				{
					rt_uint8_t cache = 0;
					Serial_State_U1_OF_M1(&cache);
				}
				Write_74HC595(NUM_74HC595, Tmpbuf); //写74HC595
			}
#endif
		}
		else if (result == RT_EOK)
		{
			rt_uint32_t rx_length;
			rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ? msg.size : sizeof(uart_rx_buffer) - 1;
			/* 读取消息*/
			rx_length = rt_device_read(msg.dev, 0, &uart_rx_buffer[0], rx_length);
			if (count + rx_length < sizeof(count_rx_buffer))
			{
				rt_memcpy(&count_rx_buffer[count], uart_rx_buffer, rx_length);
				count += rx_length;
			}
		}
		// rt_kprintf("DO_thread_entry-count %d--state %d----\n", count, Serial_State_U1_OF_M1(RT_NULL));
	}
	rt_mq_delete(rx_mq);
}

rt_uint8_t Serial_State_U1_OF_M1(rt_uint8_t *state)
{
	if (state)
	{
		g_Var_inst.U1_state = *state;
	}
	return g_Var_inst.U1_state;
}

void process_data_of_u1(unsigned char *data, rt_uint8_t len)
{
	static rt_uint8_t rec_state = 0;
	rt_uint8_t i = 0;
	do
	{
		if (rec_state == 0)
		{	
            if (*(data + i++) == 0xaa)
                rec_state++; 
		}
		else if (rec_state == 1)
		{
			if (*(data + i++) == 0x01)
			{
				rec_state = 0x5a;
				break;
			}
			else
				rec_state = 0;
		}else rec_state = 0;
	} while (i < len);

	if (rec_state == 0x5a)
	{
		Serial_State_U1_OF_M1(&rec_state);
		rec_state = 0;
	}
}
