#include "stm32f10x.h"
#include <rtthread.h>
#include <components.h>

/****************************************************
PWM channel map
PWM_AO1 <->	TIM4_CH1 PD_12
PWM_AO2	<->	TIM4_CH2 PD_13
PWM_AO3	<->	TIM4_CH3 PD_14
PWM_AO4	<->	TIM4_CH4 PD_15
PWM_AO5	<->	TIM8_CH1 PC_6
PWM_AO6	<->	TIM8_CH1 PC_7
PWM_AO7	<->	TIM8_CH1 PC_8
****************************************************/


#define AO_CHAN_MAX_NUM		7	

static void pwm_rcc_conf(void);
static void pwm_gpio_conf(void);
static void drv_pwm_conf(uint32_t freq);


/**
  * @brief  set designated channel's pwm wave duty cycle, range is [0,100]
  * @param  channel: range from 1 to 7
	* @param  duty_cycle:	range from 0%~100%, 1% step size
  * @retval 
			@arg 1: success
			@arg 0: error	
  */
uint16_t pwm_conf(uint16_t channel, uint16_t duty_cycle)
{    
	uint16_t ret = 0;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = duty_cycle*10;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;	
	
	if(duty_cycle > 100)
		return 0;
	if((channel == 0)||(channel > 7))
		return 0;	
	switch (channel)
	{
		case ( 1 ):
		{			
			TIM_OC1Init(TIM4, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
			ret = 1;
			break;
		}
		case ( 2 ):
		{			
			TIM_OC2Init(TIM4, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
			ret = 1;
			break;
		}
		case ( 3 ):
		{			
			TIM_OC3Init(TIM4, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
			ret = 1;
			break;
		}
		case ( 4 ):
		{			
			TIM_OC4Init(TIM4, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
			ret = 1;
			break;
		}
		case ( 5 ):
		{			
			TIM_OC1Init(TIM8, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
			ret = 1;
			break;
		}
		default:
		{
			ret = 0;
			break;
		}
	}
	return ret;
}


/**
  * @brief  PWM timer parameter configuration
  * @param  none
	* @param  none
  * @retval none
  */
uint16_t pwm_set_ao(uint8_t channel, uint16_t ao_data)
{
		if(channel>5)
		{
				return 0;
		}
		else 
		{
				if(ao_data>100)
				{
						return 0;						
				}
				else
				{
						pwm_conf(channel,(100-ao_data));
						return 1;
				}
		}
}

/**
  * @brief  ao driver initialization
  * @param  none
  * @retval none
  */
void drv_pwm_init(void)
{
		uint16_t i;
		pwm_rcc_conf();
		pwm_gpio_conf();
		drv_pwm_conf(100000);					//set frequency to 100Hz
		for(i=1;i<=5;i++)								//reset pwm initial state to output 0
		{
				pwm_set_ao(i,0);
		}
}

/**
  * @brief  PWM timer parameter configuration
  * @param  none
	* @param  none
  * @retval none
  */
static void drv_pwm_conf(uint32_t freq)
{	 
	uint16_t PrescalerValue = 0;
	uint16_t ccr_value = 500;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* -----------------------------------------------------------------------
    TIM8 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM8CLK frequency is set to SystemCoreClock (Hz), to get TIM8 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM8CLK / TIM8 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM8 is running at 36 KHz: TIM8 Frequency = TIM8 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM8 Channel1 duty cycle = (TIM8_CCR1/ TIM8_ARR)* 100 = 50%
    TIM8 Channel2 duty cycle = (TIM8_CCR2/ TIM8_ARR)* 100 = 37.5%
    TIM8 Channel3 duty cycle = (TIM8_CCR3/ TIM8_ARR)* 100 = 25%
    TIM8 Channel4 duty cycle = (TIM8_CCR4/ TIM8_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / freq) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = ccr_value;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	
	//PWM channel 1 init
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//PWM channel 2 init
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//PWM channel 3 init
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//PWM channel 4 init
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	
	//TIM_OCInitStructure.TIM_OutputState = TIM_OutputNState_Enable;
	//PWM channel 5 init
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);

//   TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

//	//PWM channel 6 init
//  TIM_OC2Init(TIM8, &TIM_OCInitStructure);

////   TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

//	//PWM channel 7 init
//  TIM_OC3Init(TIM8, &TIM_OCInitStructure);

////   TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM8 enable counter */
  TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}


/**
  * @brief  PWM GPIO and timer RCC	 configuration
  * @param  none
	* @param  none
  * @retval none
  */
static void pwm_rcc_conf(void)
{
  /* TIM8 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
	
  /* GPIOC and GPIOD clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE);
}

/**
  * @brief PWM GPIO configuration.
  * @param  None
  * @retval None
  */
static void pwm_gpio_conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
}


FINSH_FUNCTION_EXPORT(pwm_set_ao, reset pwm parameter.);
