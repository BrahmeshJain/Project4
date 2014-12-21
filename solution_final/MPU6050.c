#include <stdio.h>
#include "MPU6050.h"
#define FCY     40000000UL 
#include "MPU6050Wrapper.h"
#include <math.h>

void Setup_MPU6050()
{
	//Sets clock source to gyro reference w/ PLL
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
	//Sets sample rate to 1000/1+1 = 500Hz
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x30 );
	//Disable FSync, 48Hz DLPF
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG,0x00 /* 0x03 */);
	//Disable gyro self tests, scale of 500 degrees/s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);
	//Disable accel self tests, scale of +-4g, no DHPF
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0b0001000);
	//Freefall threshold of <|0mg|
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
	//Motion threshold of >0mg
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
	//Motion duration of >0s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
	//Zero motion threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
	//Disable sensor output to FIFO buffer
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);
	
	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves	
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);
 
 
 	//MPU6050_RA_I2C_MST_STATUS //Read-only
 	//Setup INT pin and AUX I2C pass through
	/* set INT INT_RD_CLEAR = 1, we will read all 14 bytes for data, which will automatically clear the interrupt status */
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x30);
	//Enable data ready interrupt
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x01);
	
	//Slave out, dont care
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00); 	
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00); 	
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00); 	
	//More slave config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00); 	
	//Motion detection control
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00); 		
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);

	//Data transfer to and from the FIFO buffer
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I 			//Read-only, I2C address
	
	printf("\nMPU6050 Setup Complete");	
}

void MPU6050_Test_I2C()
{
	unsigned char Data = 0x00;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);
	
	if(Data == 0x68)
	{
		printf("\nI2C Read Test Passed, MPU6050 Address: 0x%x", Data);
	}
	else
	{
		printf("ERROR: I2C Read Test Failed, Stopping ! MPU6050 Address: 0x%x",&Data);
	}	
}	
int MPU6050_Check_Registers()
{
	unsigned char Data = 0x00;
	unsigned char Failed = 0;
	
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, &Data, 1);
	if(Data != 0x30) { printf("\nRegister check 1 failed, value should be 0x01, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, &Data, 1);
	if(Data !=   0x00) { printf("\nRegister check 2 failed, value should be 0x03, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &Data, 1);
	if(Data != 0b00001000) { printf("\nRegister check 3 failed, value should be 0b00001000, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &Data, 1);
	if(Data != 0b00001000) { printf("\nRegister check 4 failed, value should be 0b00001000, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 5 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 6 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 7 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 8 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 9 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 10 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 11 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 12 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 13 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 14 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 15 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 16 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 17 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 18 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 19 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 20 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPUPI6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 21 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 22 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 23 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 24 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 25 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 26 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 27 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 28 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 29 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, &Data, 1);
	if(Data != 0x30) { printf("\nRegister check 30 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, &Data, 1);
	if(Data != 0x01) { printf("\nRegister check 31 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 32 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, &Data, 1); 	
	if(Data != 0x00) { printf("\nRegister check 33 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, &Data, 1); 	
	if(Data != 0x00) { printf("\nRegister check 34 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, &Data, 1); 	
	if(Data != 0x00) { printf("\nRegister check 35 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 36 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, &Data, 1); 
	if(Data != 0x00) { printf("\nRegister check 37 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, &Data, 1); 	
	if(Data != 0x00) { printf("\nRegister check 38 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &Data, 1);
	if(Data != 0x00) { printf("\nRegister 39 check failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, &Data, 1);
	if(Data != 0x02) { printf("\nRegister check 40 failed, value should be 0x02, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 41 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, &Data, 1);
	if(Data != 0x00) { printf("\nRegister check 42 failed, value should be 0x00, was 0x%x", Data); Failed = 1; }
	
	if (Failed == 0) { printf("\nRegister value check passed"); }
	else { printf("\nRegister value check failed");}
	
	return(Failed);
}
