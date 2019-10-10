#include "main.h"
#include "stm32f1xx_hal.h"
#include "led.h"
#include "motor.h"
#include "servo.h"
#include "ps2.h"
//#include "searchpath.h"

//extern mt_ctrltype MT_CTRL;
//extern pidtype MT_MTS[4];
//extern SPI_HandleTypeDef SPIPORT;
//extern int16_t SPEED_XYR[3];
//extern uint8_t CAL_SPEED;
uint8_t ps2Txbuf[BUFFSIZE]={0x01,0x42,0xff,0xff,0xff,0xff,0xff,0xff,0xff},ps2Rxbuf[BUFFSIZE]={0};
uint8_t ps2_busy = 0;
uint8_t lastmode = 0;
uint8_t ctrl_des =0;//car 0 robot_arm 1
uint8_t key[16]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};//ps2 key

uint16_t servo1=1500;
uint16_t servo2=1500;
uint16_t servo3=1500;

static void uDelay(uint32_t t)
{
	uint32_t i;
	for(i=0;i<t;++i)
		__nop();
}

void Button_Servo(uint8_t button1,uint8_t button2,uint16_t *servo,uint8_t rate)
{
		if(button1==0)
			{
				if(*servo<2500)
					*servo+=rate;
				else
					*servo=2500;

			}
			
		if(button2==0)
			{
				if(*servo>500)
					*servo-=rate;
				else
					*servo=500;
			}
}
	

void Ps2_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
{

	static uint32_t time_next = 0;
	uint32_t time_curr = HAL_GetTick();
	if(time_curr <= time_next)
		return;
	if(ps2_busy == 0)
	{

		PS2_ReadData();
		
		/*slove ps2 data*/
		if((ps2Rxbuf[1]==0x73)&&(ps2Rxbuf[2]==0x5a))
		{
			//mode0
			/*if(lastmode != 0)
			{
				Motor_PID_Enable(ctrl,mt);
				lastmode = 0;
			}*/
			
					
			key[PSB_L3]=(ps2Rxbuf[3]>>(PSB_L3-1))&0x01;
			key[PSB_R3]=(ps2Rxbuf[3]>>(PSB_R3-1))&0x01;
			key[PSB_PAD_UP]=(ps2Rxbuf[3]>>(PSB_PAD_UP-1))&0x01;
			key[PSB_PAD_RIGHT]=(ps2Rxbuf[3]>>(PSB_PAD_RIGHT-1))&0x01;
			key[PSB_PAD_DOWN]=(ps2Rxbuf[3]>>(PSB_PAD_DOWN-1))&0x01;
			key[PSB_PAD_LEFT]=(ps2Rxbuf[3]>>(PSB_PAD_LEFT-1))&0x01;
			
			key[PSB_L2]=(ps2Rxbuf[4]>>(PSB_L2-9))&0x01;
			key[PSB_R2]=(ps2Rxbuf[4]>>(PSB_R2-9))&0x01;
			key[PSB_L1]=(ps2Rxbuf[4]>>(PSB_L1-9))&0x01;
			key[PSB_R1]=(ps2Rxbuf[4]>>(PSB_R1-9))&0x01;
			
			Button_Servo(key[PSB_L1],key[PSB_L2],&servo1,10);
			Set_Servo(1,servo1);
			Button_Servo(key[PSB_PAD_UP],key[PSB_PAD_DOWN],&servo2,10);
			Set_Servo(2,servo2);
			Button_Servo(key[PSB_PAD_LEFT],key[PSB_PAD_RIGHT],&servo3,10);
			Set_Servo(3,servo3);
		
			
			speed->y=(ps2Rxbuf[7]<<3)-1023;//y
			speed->x=-(ps2Rxbuf[8]<<3)+1023;
			speed->r=(ps2Rxbuf[5]<<3)-1023;

			speed->y = (speed->y <= JOY_TH && speed->y >= -JOY_TH)?0:speed->y;
			speed->x = (speed->x <= JOY_TH && speed->x >= -JOY_TH)?0:speed->x;
			speed->r = (speed->r <= JOY_TH && speed->r >= -JOY_TH)?0:speed->r;

			speed->cal_speed = 1;

			

			
	
			
			//Motor_unlock(&MT_CTRL);
			//Set_blink(1,6);//ms
		}
		else if((ps2Rxbuf[1]==0x41)&&(ps2Rxbuf[2]==0x5a))
		{
			//mode1
			//Searchpath(SPEED_XYR,mt);
			//CAL_SPEED = 1;
		}
		/*else 
		{
			Motor_lock(&MT_CTRL);
		}*/
		time_next = time_curr + 10;
		//ps2_busy = 1;
	}
}

void PS2_ReadData(void)
{
		uint8_t i,j;
		uint8_t tmp,send;
		/*read data from ps2*/
		//HAL_SPI_TransmitReceive_DMA(&SPIPORT,Txbuf,Rxbuf,BUFFSIZE);
		LL_GPIO_ResetOutputPin(ENBANK, ENPORT);
		uDelay(1);
		for(i=0;i<BUFFSIZE;++i)
		{
			tmp = 0;
			send = ps2Txbuf[i];
			if((send & 0x01) == 0)
				{
					LL_GPIO_ResetOutputPin(SPIBANK, CMD);
				}
				else
				{
					LL_GPIO_SetOutputPin(SPIBANK, CMD);
				}
				for(j=0;j<8;++j)
			{
				tmp >>= 1;
				send >>= 1;
				LL_GPIO_ResetOutputPin(SPIBANK, SCK);
				uDelay(1);
				if(GPIO_PIN_SET == HAL_GPIO_ReadPin(SPIBANK,DAT))
					tmp |= 0x80;
				uDelay(71);
				LL_GPIO_SetOutputPin(SPIBANK, SCK);
				if((send & 0x01) == 0)
				{
					LL_GPIO_ResetOutputPin(SPIBANK, CMD);
				}
				else
				{
					LL_GPIO_SetOutputPin(SPIBANK, CMD);
				}
				
				uDelay(74);
			}
			ps2Rxbuf[i] = tmp;
			uDelay(70);
		}
		LL_GPIO_SetOutputPin(ENBANK, ENPORT);

}