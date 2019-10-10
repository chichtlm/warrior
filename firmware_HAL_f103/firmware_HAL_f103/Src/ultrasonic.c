#include "main.h"
#include "stm32f1xx_hal.h"
#include "ultrasonic.h"

//uint32_t time_begin;
uint32_t time_len;
uint8_t us_en = 0;
static void uDelay(uint32_t t)
{
	uint32_t i;
	for(i=0;i<t;++i)
		__nop();
}

__STATIC_INLINE void Gentrig(void)
{
	LL_GPIO_SetOutputPin(US_BANK, US_TRIG);
	uDelay(105);
	LL_GPIO_ResetOutputPin(US_BANK, US_TRIG);
}

void Msr_dist_task(void)
{
	static uint32_t time_next = 0;
	uint32_t time_curr ;
	if(us_en == 0)
		return;
	time_curr = HAL_GetTick();
	if(time_curr >= time_next)
	{
		Gentrig();
		time_next = time_curr + 60;
	}
}

void EXTI1_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(US_ECHO) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(US_ECHO);
		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(US_BANK,US_ECHO))
		{	
			LL_TIM_EnableCounter(USTIM);
			//LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
		}
		else
		{
			LL_TIM_DisableCounter(USTIM);
			//LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
			time_len = LL_TIM_GetCounter(USTIM);
			LL_TIM_SetCounter(USTIM, 0);
		}	
	}		
}
