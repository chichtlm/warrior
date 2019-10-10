#include "main.h"
#include "stm32f1xx_hal.h"
#include "mpu9250.h"

extern I2C_HandleTypeDef HIIC;

void Read_MPU9250_acc(imutype *imudata)
{ 
	/*accel*/
  I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_H,(uint8_t *)&imudata->accel_xout+1,1);
	I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_L,(uint8_t *)&imudata->accel_xout,1);
  imudata->accel_xout/=164; 						  

	I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_H,(uint8_t *)&imudata->accel_yout+1,1);
	I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_L,(uint8_t *)&imudata->accel_yout,1);
	imudata->accel_yout/=164; 
	
	I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_H,(uint8_t *)&imudata->accel_zout+1,1);
	I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_L,(uint8_t *)&imudata->accel_zout,1);
  imudata->accel_zout/=164; 				  
}

void Read_MPU9250_gyro(imutype *imudata)
{ 
	/*gyro*/
	I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_H,(uint8_t *)&imudata->gyro_xout+1,1);
	I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_L,(uint8_t *)&imudata->gyro_xout,1);
  imudata->gyro_xout/=16.4; 

	I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_H,(uint8_t *)&imudata->gyro_yout+1,1);
	I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_L,(uint8_t *)&imudata->gyro_yout,1);
  imudata->gyro_yout/=16.4; 		

	I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_H,(uint8_t *)&imudata->gyro_zout+1,1);
	I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_L,(uint8_t *)&imudata->gyro_zout,1);
  imudata->gyro_zout/=16.4; 	
}	
	
void Read_MPU9250_mag(imutype *imudata)
{ 	
  /*mag*/
	uint8_t endbyte;
	I2C_ByteRead(MAG_ADDRESS,MAG_XOUT_L,(uint8_t *)&imudata->mag_xout,2);
	I2C_ByteRead(MAG_ADDRESS,MAG_YOUT_L,(uint8_t *)&imudata->mag_yout,2);
	I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_L,(uint8_t *)&imudata->mag_zout,2);
	I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_H+1,&endbyte,1);
}

/** mpu6050≥ı ºªØ***/  

void InitMPU6050(void)
{
	I2C_ByteWrite(SlaveAddress,PWR_MGMT_1,0x00);
	I2C_ByteWrite(SlaveAddress,SMPLRT_DIV,0x07);
	I2C_ByteWrite(SlaveAddress,CONFIG,0x06);
	I2C_ByteWrite(SlaveAddress,GYRO_CONFIG,0x18);
	I2C_ByteWrite(SlaveAddress,ACCEL_CONFIG,0x01);

	I2C_ByteWrite(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
	HAL_Delay(10);	
	I2C_ByteWrite(MAG_ADDRESS,0x0A,0x06);
	HAL_Delay(10);	
}
/*
int16_t GetData(unsigned char REG_Address)
{
    char H;//,L;
    H=I2C_ByteRead(REG_Address,2);
    L=I2C_ByteRead(REG_Address+1);
    return H;//<<8)+L;   
}*/
