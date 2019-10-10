#include "main.h"
#include "stm32f1xx_hal.h"
#include "config.h"
#include "battery.h"
#include "motor.h"
#include "oled.h"
#include "led.h"

uint16_t voltbuf[5] = {4095,4095,4095,4095,4095};

extern ADC_HandleTypeDef HADC;

uint8_t Check_volt(uint16_t *volt)
{
	uint8_t i,cnt=0;
	uint32_t vol = 0;
	static uint32_t time_next = 0;
	uint32_t time_curr = HAL_GetTick();
	if(time_curr >= time_next)
	{
		HAL_ADC_Start_IT(&HADC);
		time_next = time_curr + BAT_PERIOD;
	}
	for(i=0;i<5;++i)
	{
		vol += voltbuf[i];
		if(voltbuf[i] < BAT_TH)
			++cnt;
	}
	*volt = vol / 5;
	if(cnt >= 3)
		return 0xff;
	else 
		return 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint8_t i = 0;
	voltbuf[i] = HAL_ADC_GetValue(hadc);
	i = (i==5? 0:++i);
}


