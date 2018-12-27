#ifndef	__DIO
#define __DIO
#include <components.h>
#include <rtthread.h>
#include "dio_bsp.h"

#define	DI_MAX_CNT						16
#define DIN_MASK_MASK         0x003f
#define DIN_MASK_MASK1        0x3fff //ÆÁ±Î
#define DIN_POLARITY_MASK     0xffc0
#define DI_BUF_DEPTH 					48
#define	DI_UPDATE_PERIOD			1000
#define SAMPLE_INTERVAL				6
//RT_TIMER_TICK_PER_SECOND
typedef struct
{
		uint16_t	bitmap;
		uint16_t	reg_array[DI_BUF_DEPTH];		
}di_dev_st;

typedef struct
{
		uint16_t	bitmap;
}do_dev_st;

typedef struct
{
		di_dev_st	din;
		do_dev_st	dout;		
}dio_dev_st;

typedef struct
{
		uint16_t 		pin_id;
		void*  			pin_base;
}pin_map_st;

static uint16_t do_set(int16_t pin_id, BitAction value);
const pin_map_st in_pin_map_inst[16]=
{
		{GPIO_Pin_8, 		GPIOA},		//DI1
		{GPIO_Pin_9, 		GPIOC},		//DI2
		{GPIO_Pin_8, 		GPIOC},		//DI3
		{GPIO_Pin_7, 		GPIOC},		//DI4
		{GPIO_Pin_11, 	GPIOD},		//DI5
		{GPIO_Pin_15, 	GPIOB},		//DI6
		{GPIO_Pin_14, 	GPIOB},		//DI7
		{GPIO_Pin_13, 	GPIOB},		//DI8
		{GPIO_Pin_12, 	GPIOB},		//DI9		
		{GPIO_Pin_14, 	GPIOE},		//DI10
		{GPIO_Pin_13, 	GPIOE},		//DI11
		{GPIO_Pin_12, 	GPIOE},		//DI12
		{GPIO_Pin_11, 	GPIOE},		//DI13		
		{GPIO_Pin_1, 		GPIOB},		//DI14
		{GPIO_Pin_0, 		GPIOB},		//DI15		
		{GPIO_Pin_5, 		GPIOC},		//DI16		
};

const pin_map_st out_pin_map_inst[16]=
{
		{GPIO_Pin_11, 	GPIOA},		//DO1
		{GPIO_Pin_12, 	GPIOA},		//DO2
		{GPIO_Pin_0, 		GPIOD},		//DO3
		{GPIO_Pin_1, 		GPIOD},		//DO4
		{GPIO_Pin_4, 		GPIOD},		//DO5
		{GPIO_Pin_5, 		GPIOD},		//DO6
		{GPIO_Pin_6, 		GPIOD},		//DO7
		{GPIO_Pin_7, 		GPIOD},		//DO8
		{GPIO_Pin_4, 		GPIOB},		//DO9
		{GPIO_Pin_6, 		GPIOB},		//DO10
		{GPIO_Pin_7, 		GPIOB},		//DO11
		{GPIO_Pin_0, 		GPIOE},		//DO12
		{GPIO_Pin_1, 		GPIOE},		//DO13
		{GPIO_Pin_2, 		GPIOE},		//DO14
		{GPIO_Pin_3, 		GPIOE},		//DO15
		{GPIO_Pin_4, 		GPIOE},		//DO16
};

//local variable definition
static dio_dev_st dio_dev_inst;

static uint16_t	drv_di_timer_init(void);
static void dio_reg_init(void);
static void di_reg_update(void);
static void drv_dio_bsp_init(void);

//digital input sampling thread
void di_thread_entry(void* parameter)
{	
		return;
		rt_thread_delay(DI_THREAD_DELAY);
		drv_dio_bsp_init();
		dio_reg_init();
		drv_di_timer_init();
		rt_thread_delay(300);
		while(1)
		{
				di_reg_update();
				rt_thread_delay(DI_UPDATE_PERIOD);
		}
}


/**
  * @brief  digital IOs GPIO initialization
  * @param  none
  * @retval none
  */
static void drv_dio_bsp_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t i;
			
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		for(i=0;i<=15;i++)
		{
				GPIO_InitStructure.GPIO_Pin = in_pin_map_inst[i].pin_id;
				GPIO_Init(in_pin_map_inst[i].pin_base, &GPIO_InitStructure);	
		}

		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		
		for(i=0;i<=15;i++)
		{
				GPIO_InitStructure.GPIO_Pin = out_pin_map_inst[i].pin_id;
				GPIO_Init(out_pin_map_inst[i].pin_base, &GPIO_InitStructure);	
		}	

		for(i=1;i<=16;i++)
		{
				do_set(i,Bit_RESET);	
		}		
}


/**
  * @brief  digital io stucture initialization
  * @param  none
  * @retval none
  */
static void dio_reg_init(void)
{
		uint16_t i;
		dio_dev_inst.din.bitmap = 0;
		for(i=0;i<DI_BUF_DEPTH;i++)
		{
				dio_dev_inst.din.reg_array[i] = 0;
		}
		dio_dev_inst.dout.bitmap = 0;
}

/**
  * @brief  digital input result caculation
  * @param  none
  * @retval none
  */
static void di_reg_update(void)
{
		uint16_t	di_data,i,j;
		uint16_t 	di_reg[DI_MAX_CNT];
		di_data = 0;
		for(i=0;i<DI_MAX_CNT;i++)
		{
				di_reg[i] = 0;
		}
		for(i=0;i<DI_MAX_CNT;i++)//outer loop caculate each channels di data
		{
				for(j=0;j<DI_BUF_DEPTH;j++)//inner loop caculate sum of one channel di data
				{
						di_reg[i] += (dio_dev_inst.din.reg_array[j]>>i)&(0x0001);
				}
		}
		for(i=0;i<DI_MAX_CNT;i++)
		{
				//if((di_reg[i]>(DI_BUF_DEPTH>>2))&&(di_reg[i]<(DI_BUF_DEPTH-(DI_BUF_DEPTH>>2))))//[25%~75%] duty cycle is consider set state, otherwise is considered reset state
				if(di_reg[i]<(DI_BUF_DEPTH-(DI_BUF_DEPTH>>2)))//[0~75%] duty cycle is consider set state, otherwise is considered reset state
				{
						di_data |= (0x0001<<i);
				}
				else
				{
						di_data &= ~(0x0001<<i);
				}
		}
		dio_dev_inst.din.bitmap = di_data;
}


/**
  * @brief  raw digital input data read
  * @param  none
  * @retval 16 channels data, each bit stands for one channel
  */
static uint16_t di_read(void)
{
		uint16_t read_bitmap;
		uint16_t i;
		read_bitmap = 0;
		for(i=0;i<=15;i++)
		{
				read_bitmap |= GPIO_ReadInputDataBit(in_pin_map_inst[15-i].pin_base,in_pin_map_inst[15-i].pin_id);
				if(i<15)
				{
						read_bitmap = read_bitmap << 1;
				}
		}	
		return read_bitmap;
}

static uint16_t do_set(int16_t pin_id, BitAction value)
{
		if((pin_id <= 16)&&(pin_id > 0))
		{
				GPIO_WriteBit(out_pin_map_inst[pin_id-1].pin_base,out_pin_map_inst[pin_id-1].pin_id,value);
				return 1;
		}
		else
		{
				return 0;
		}
}


static void do_set_all(void)
{
		uint16_t i;
		for(i=1;i<=16;i++)
		{
				do_set(i,Bit_SET);
		}
}

static void do_reset_all(void)
{
		uint16_t i;
		for(i=1;i<=16;i++)
		{
				do_set(i,Bit_RESET);
		}
}


/**
  * @brief  digital input sample interval timeout callback function, calls di_read() each time to update di buffer queue
  * @param  none
  * @retval none
  */
static void stimer_di_timeout(void* parameter)
{
		static uint16_t count = 0;
		if(count >= DI_BUF_DEPTH)
		{
				count = count%DI_BUF_DEPTH;
		}
		dio_dev_inst.din.reg_array[count] = di_read();
		count++;
}

/**
  * @brief  digital input sample interval timer initialization, expires in 6 miliseconds pieriod
  * @param  none
  * @retval none
  */
static uint16_t	drv_di_timer_init(void)
{
		rt_timer_t stimer_dio;
		stimer_dio = rt_timer_create("stimer_di", 
									stimer_di_timeout, 
									RT_NULL, 
									SAMPLE_INTERVAL,
									RT_TIMER_FLAG_PERIODIC); 
		rt_timer_start(stimer_dio);
		return 1;
}
/**
  * @brief  digital IO initialization function
  * @param  none
  * @retval none
 */
void drv_dio_init(void)
{
// 	drv_dio_bsp_init();

//	drv_di_timer_init();
}


/**
  * @brief  update global variable g_din_inst and g_ain_inst according to di and ai inputs
  * @param  none
  * @retval none
**/	
void di_sts_update(sys_reg_st*	gds_sys_ptr)
{
		uint16_t din_mask_bitmap;
		uint16_t din_bitmap_polarity;
	
		din_mask_bitmap = (gds_sys_ptr->config.dev_mask.din[0]|(DIN_MASK_MASK))&(DIN_MASK_MASK1);

	  din_bitmap_polarity = gds_sys_ptr->config.dev_mask.din_bitmap_polarity[0]&(DIN_POLARITY_MASK);
	  //mask±¨¾¯ÑÚÂë
		dio_dev_inst.din.bitmap = (~(dio_dev_inst.din.bitmap^din_bitmap_polarity));
		// Êý×ÖÊäÈëÑÚÂë
		gds_sys_ptr->status.din_bitmap[0] = din_mask_bitmap & dio_dev_inst.din.bitmap;
}

void dio_set_do(uint16_t channel_id, BitAction data)
{
		do_set(channel_id,data);
}



FINSH_FUNCTION_EXPORT(do_set, set data out bit);
FINSH_FUNCTION_EXPORT(do_set_all, set all data bit 1);
FINSH_FUNCTION_EXPORT(do_reset_all, set all data out bit 0);
#endif  //__DIO

