#include "main.h"
#include "stm32f1xx_hal.h"
#include "config.h"
#include "motor.h"
#include "bluetooth.h"

extern UART_HandleTypeDef UART_PORT;
uint8_t bt_cnt = 0;
uint8_t btRxbuf[10]={0};

void BT_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/		
	switch(bt_cnt)
	{
		case 0:
			btRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&UART_PORT,btRxbuf,1);
			bt_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&UART_PORT,&btRxbuf[1],9);
			bt_cnt = 3;
		break;
			case 3:
		break;
		case 4:
			if(0xfe == btRxbuf[1] && 0x01 == btRxbuf[2] && 0x00 == btRxbuf[9])
			{ 
				speed->y = *(int16_t *)&btRxbuf[3];//y
				speed->x = *(int16_t *)&btRxbuf[5];
				speed->r = -*(int16_t *)&btRxbuf[7];
				speed->cal_speed = 1;
				speed->time = HAL_GetTick();
			}
			bt_cnt = 0;
			break;
		default:
			bt_cnt = 0;		
	}
	
	#if OFFLINE_DECT == 1
	if(speed->time+TIMEOUT_TH <=  HAL_GetTick())
	{
		speed->y = 0;
		speed->x = 0;
		speed->r = 0;
		speed->cal_speed = 1;
		speed->time = HAL_GetTick();
	}
	#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &UART_PORT)
	{
			switch(bt_cnt)
		{
			case 1:
			if(0xff == btRxbuf[0])
				bt_cnt = 2;
			else
				bt_cnt = 0;
			break;
			case 3:
				bt_cnt = 4;
			break;
		}
	}
}
