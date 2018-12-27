#include <stm32f10x_conf.h>
#include "sys_conf.h"
#include "Usr_Porttimer.h"
#include "Usr_Portserial.h"
#include <rtdevice.h>


void Comm_PC_PortTimersInit(USHORT usTim1Timerout50us)
{

	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//====================================时钟初始化===========================
	//使能定时器3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//====================================定时器初始化===========================
	//定时器时间基配置说明
	//HCLK为72MHz，APB1经过2分频为36MHz
	//TIM3的时钟倍频后为72MHz（硬件自动倍频,达到最大）
	//TIM3的分频系数为3599，时间基频率为72 / (1 + Prescaler) = 20KHz,基准为50us
	//TIM最大计数值为usTim1Timerout50u
	
	PrescalerValue = (uint16_t) (SystemCoreClock / 20000) - 1;
	//定时器1初始化
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) usTim1Timerout50us;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	//预装载使能
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	//====================================中断初始化===========================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	//清除溢出中断标志位
//	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
//	//定时器3溢出中断关闭
//	TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
//	//定时器3禁能
//	TIM_Cmd(TIM5 ,DISABLE);
	
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, DISABLE);
	return;
}

void Comm_PC_PortTimersEnable(void)
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, ENABLE);
}

void Comm_PC_PortTimersDisable(void)
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, DISABLE);
}



void TIM5_IRQHandler(void)
{
	rt_interrupt_enter();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);	     		//清中断标记
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);		//清除定时器T5溢出中断标志位
		Comm_PC_inst.rx_timeout = 1;
	}
	rt_interrupt_leave();
}



BOOL Comm_T_PortTimersInit(USHORT usTimeOut50us)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	static USHORT usT35TimeOut50us;
	static USHORT usPrescalerValue = 0;

	//====================================时钟初始化===========================
	//使能定时器2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//====================================定时器初始化===========================
	//定时器时间基配置说明
	//HCLK为72MHz，APB1经过2分频为36MHz
	//TIM2的时钟倍频后为72MHz（硬件自动倍频,达到最大）
	//TIM2的分频系数为3599，时间基频率为72 / (1 + Prescaler) = 20KHz,基准为50us
	//TIM最大计数值为usTim1Timerout50u	
	usPrescalerValue = (uint16_t) (SystemCoreClock / 20000) - 1;
	//保存T35定时器计数值
	usT35TimeOut50us = usTimeOut50us; 

	TIM_TimeBaseStructure.TIM_Prescaler = usPrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) usT35TimeOut50us;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//预装载使能
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	//====================================中断初始化===========================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	//清除溢出中断标志位
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	//定时器3溢出中断关闭
//	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
//	//定时器3禁能
//	TIM_Cmd(TIM2, DISABLE);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);

	return TRUE;
}

void Comm_T_PortTimersEnable(void)
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);
}

void Comm_T_PortTimersDisable(void)
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler(void)
{
	rt_interrupt_enter();
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);	     //清中断标记
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	 //清除定时器TIM2溢出中断标志位
//		prvvTIMERExpiredISR();
	}
	rt_interrupt_leave();
}


