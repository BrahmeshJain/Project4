/* *********************************************************************
 *
 * User level program 
 *
 * Program Name:        Gyroscope and accelerometer sensing and filter
 * Target:              Intel Galileo Gen1
 * Architecture:		x86
 * Compiler:            i586-poky-linux-gcc
 * File version:        v1.0.0
 * Author:              Brahmesh S D Jain
 * Email Id:            Brahmesh.Jain@asu.edu
 **********************************************************************/

/* *************** INCLUDE DIRECTIVES FOR STANDARD HEADERS ************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include "MPU6050Wrapper.h"
#include "MPU6050.h"
#include <pthread.h>
#include <linux/input.h>
#include <math.h>
#include "kalman.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <poll.h>

#define IP_ADDRESS										"169.254.147.71"
#define PORT_NUMBER										9999
#define I2C_SENSOR_DEVICE_ADDRESS						0x69


#define WINDOW											100
#define NO_ACCELERATION_COUNT							5
#define PI												3.14159265
#define ANGLE_CALIBRATION								500.0
#define POSITION_CALIBRATION							20000.0
#define DATA_READY										1
#define DATA_PROCESSED									0
#define CPU_FREQ										400000000.0
#define REDUCTION_FACTOR_GYRO							1
#define gyro_xsensitivity 								(66.5 * REDUCTION_FACTOR_GYRO)
#define gyro_ysensitivity 								(66.5 * REDUCTION_FACTOR_GYRO)
#define gyro_zsensitivity 								(65.5 * REDUCTION_FACTOR_GYRO)
#define AVERAGING_COUNT									5
#define REDUCTION_FACTOR								0.05
#define CALIBRATION_COUNT								5000
#define ANGULAR_INTEGRATION								0	
#define KALMAN_FILTERING								1
#define LEFT_BUTTON										0
#define RIGHT_BUTTON									1
#define GYRO_SCALING_FACTOR								10000

//#define DEBUG

int FileDesc,sockfd;
struct sockaddr_in servaddr;
/* 
 * The following variables dont need mutex, because they either dont 
 * contend or Atomic instruction has been used to update them
 */
volatile int DataReady = DATA_PROCESSED;
volatile unsigned char position_reset = 0;
volatile unsigned char filtering_type = ANGULAR_INTEGRATION;
volatile unsigned char ButtonPressed = LEFT_BUTTON;
/*
 * Does not need mutex for below , as threads does not contend for 
 * these variables
 */
short Accel_X_Offset = 0, Accel_Y_Offset = 0, Accel_Z_Offset = 0;
short Gyro_X_Offset = 0, Gyro_Y_Offset = 0, Gyro_Z_Offset = 0;

/*
 * Assembly level read Time stamp counter
 */
static __inline__ unsigned long long rdtsc(void)
{
    unsigned long long int x;
    __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
    return x;
}

/* *******************************************************************************************
 * NAME:             sys_futex (reference : http://locklessinc.com/articles/futex_cheat_sheet/)
 * CALLED BY:        DataProcessingThread and DataAcquisitionThread
 * DESCRIPTION:      Will make DataProcessingThreadto sleep till the 
 *                   data is ready from the sensor
 * INPUT PARAMETERS: http://locklessinc.com/articles/futex_cheat_sheet/
 * RETURN VALUES:    success(0)/fail(-1)
 ***********************************************************************************************/
static long sys_futex(void *addr1, int op, int val1, struct timespec *timeout, void *addr2, int val3)
{
	return syscall(SYS_futex, addr1, op, val1, timeout, addr2, val3);
}
/* *********************************************************************
 * NAME:             SensorInit
 * CALLED BY:        Called by main() thread
 * DESCRIPTION:      Main thread to intialize, gpios and files
 * INPUT PARAMETERS: None
 * RETURN VALUES:    None
 ***********************************************************************/
void SensorInit(void)
{
	int I2cMux;
	I2cMux = open("/sys/class/gpio/export", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n Gpio export failed ");
	}

	write(I2cMux,"29",2);
	write(I2cMux,"30",2);
    write(I2cMux,"15",2);
	close(I2cMux);
    /* Set the direction for gpios */
	I2cMux = open("/sys/class/gpio/gpio29/direction", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n gpio29  Failed : open");
	}
	write(I2cMux,"out",3);
    close(I2cMux);

    /* Set the direction for gpios */
	I2cMux = open("/sys/class/gpio/gpio30/direction", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n gpio30  Failed : open");
	}
	write(I2cMux,"out",3);
    close(I2cMux);

    /* Set the direction for gpios */
	I2cMux = open("/sys/class/gpio/gpio15/direction", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n gpio15  Failed : open");
	}
	write(I2cMux,"in",2);
    close(I2cMux);

    /* Set values for all the gpios */
	I2cMux = open("/sys/class/gpio/gpio29/value", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n gpio29  Failed : value");
	}
	write(I2cMux,"0",1);
    close(I2cMux);

    /* Set values for all the gpios */
	I2cMux = open("/sys/class/gpio/gpio30/value", O_WRONLY);
	if (I2cMux < 0)
	{
		printf("\n gpio30  Failed : value");
	}
	write(I2cMux,"0",1);
    close(I2cMux);

	FileDesc = open("/dev/i2c-0",O_RDWR);
	if (FileDesc < 0)
	{
		printf("\n i2c-dev driver file open failed");
	}
	
	if (ioctl(FileDesc,I2C_SLAVE,I2C_SENSOR_DEVICE_ADDRESS) < 0)
	{
		printf("\n i2c slave sensor device file open failed");
	}
	/* Intialize TCP */
   sockfd = socket(AF_INET,SOCK_STREAM,0);
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr = inet_addr(IP_ADDRESS);
   servaddr.sin_port = htons(PORT_NUMBER);
   if (connect(sockfd,(struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
   {
	   printf("\n TCP(Unit Proj) server DOWN !!\n");
	   exit(EXIT_FAILURE);
   }
   printf("\n Connected to Unit Proj server \n");
}

/* *********************************************************************
 * NAME:             SensorWrite
 * CALLED BY:        Whenever the I2C transaction needs to be taken
 * DESCRIPTION:      Writes to i2c device file
 * INPUT PARAMETERS: RegAddress = Address of Reg in the sensor
 *                   Value = Value to be written to that register
 * RETURN VALUES:    None
 ***********************************************************************/
void SensorWrite(unsigned char RegAddress, unsigned char Value)
{
	unsigned char buf[2] = {RegAddress,Value};
	
	if (2 != write(FileDesc, buf, 2))
    {
		printf("\n i2c slave sensor device write register failed");
	}
}
/* *********************************************************************
 * NAME:             SensorRead
 * CALLED BY:        Whenever the I2C transaction needs to be taken
 * DESCRIPTION:      Reads to i2c device file
 * INPUT PARAMETERS: RegAddress = Address of Reg in the sensor
 *                   buff = pointer to buffer to which the data is written
 *                   size = number if bytes written
 * RETURN VALUES:    None
 ***********************************************************************/
void SensorRead(unsigned char RegAddress, unsigned char* buff , unsigned short size)
{
	unsigned char SetAdd = RegAddress;
	
	if (1 != write(FileDesc, &SetAdd, 1))
    {
		printf("\n i2c slave sensor device Failed to set address for reading");
	}
	if (size != read(FileDesc, buff, size))
	{
		printf("\n i2c slave sensor device failed to read ");
	}
}
/* *********************************************************************
 * NAME:             DataProcessingThread
 * CALLED BY:        Started by main() thread
 * DESCRIPTION:      Reads 14 bytes sensor data, process it and sends the
 *                   result to the TCP server
 * INPUT PARAMETERS: dummy not used
 * RETURN VALUES:    None
 ***********************************************************************/
void* DataProcessingThread(void* dummy)
{
	unsigned char count2 = 0;
	unsigned char DataAcquired[14], test;
	int Accel_X_sum = 0, Accel_Y_sum = 0, Accel_Z_sum = 0;
	int Gyro_X_sum = 0, Gyro_Y_sum = 0, Gyro_Z_sum = 0;
	unsigned long long PreviousTimestamp = 0;
	short accelerationx[2] = {0};
	short accelerationy[2] = {0};
	short accelerationz[2] = {0};

	int velocityx[2] = {0};
	int velocityy[2] = {0};
	int velocityz[2] = {0};

	int PreVelocityx = {0};
	int PreVelocityy = {0};
	int PreVelocityz = {0};

	int positionX[2] = {0};
	int positionY[2] = {0};
	int positionZ[2] = {0};

	int PrePositionX = 0;
	int PrePositionY = 0;
	int PrePositionZ = 0;

	unsigned char countx = 0;
	unsigned char county = 0;
	unsigned char countz = 0;

	float Gyro_X_Rate[2] = {0};
	float Gyro_Y_Rate[2] = {0};
	float Gyro_Z_Rate[2] = {0};
	
	float Gyro_X_Angle[2] = {0};
	float Gyro_Y_Angle[2] = {0};
	float Gyro_Z_Angle[2] = {0};
	
	float Kalman_X_Angle = 0;
	float Kalman_Y_Angle = 0;
	float Kalman_Z_Angle = 0;
	
	float DelT = 0.025;

	char sendline[700];
	
	/* Required for displaying */
	float Pre_Gyro_Xangle = 0;
	float Pre_Gyro_Yangle = 0;
	float Pre_Gyro_Zangle = 0;
	
	float Gyro_X_Angle_L = Pre_Gyro_Xangle;
	float Gyro_Y_Angle_L = Pre_Gyro_Yangle;
	float Gyro_Z_Angle_L = Pre_Gyro_Zangle;

    /* Initialize for the first time */
    PreviousTimestamp = rdtsc();
    /* Clear off the interrupt line once u=by dummy read */
    while(1)
    {
		while(count2 < AVERAGING_COUNT)
		{
			count2++;
			/* go to sleep till the data ready interrupt arrive */
			sys_futex((void*)&DataReady,FUTEX_WAIT,DATA_PROCESSED,NULL,NULL,0);
			/* read 14 bytes data having gyro , acceleration, temperature */
			SensorRead(MPU6050_RA_ACCEL_XOUT_H,&DataAcquired[0],14);
			/* Make the Data ready  variable processed */
			test = __sync_sub_and_fetch(&DataReady,1);
#ifdef DEBUG
			printf(" Test = %d",test);
#endif
			/* decode the data */
			Accel_X_sum += (short)((DataAcquired[0]<<8)|DataAcquired[1]);
			Accel_Y_sum += (short)((DataAcquired[2]<<8)|DataAcquired[3]);
			Accel_Z_sum += ((DataAcquired[4]<<8)|DataAcquired[5]);
			
			Gyro_X_sum += (short)((DataAcquired[8]<<8)|DataAcquired[9]);
			Gyro_Y_sum += (short)((DataAcquired[10]<<8)|DataAcquired[11]);
			Gyro_Z_sum += (short)((DataAcquired[12]<<8)|DataAcquired[13]);
		}
		count2 = 0;
		/* Caculate delta time */
		DelT = (float)((rdtsc() - PreviousTimestamp) / CPU_FREQ);
#ifdef DEBUG
		printf("\nDelta T = %f",DelT);
#endif
		PreviousTimestamp = rdtsc();
#ifdef DEBUG
		printf("\nTime stamp = %llu",PreviousTimestamp);
#endif
        // Reffered Freescale Semiconductor Application Note "Implementing Positioning Algorithms Using Accelerometers"
        // to write the below code
		// 10 sums of the acceleration sample
		Accel_X_sum = Accel_X_sum / AVERAGING_COUNT;
		Accel_Y_sum = Accel_Y_sum / AVERAGING_COUNT;
		Accel_Z_sum = Accel_Z_sum / AVERAGING_COUNT;

		Gyro_X_sum = Gyro_X_sum / AVERAGING_COUNT;
		Gyro_Y_sum = Gyro_Y_sum / AVERAGING_COUNT;
		Gyro_Z_sum = Gyro_Z_sum / AVERAGING_COUNT;

		// division by 64
		accelerationx[1] = (short)(Accel_X_sum - Accel_X_Offset); //eliminating zero reference
																//offset of the acceleration data
		accelerationy[1] = (short)(Accel_Y_sum - Accel_Y_Offset); // to obtain positive and negative
															   //acceleration
		accelerationz[1] = (short)(Accel_Z_sum - Accel_Z_Offset); // to obtain positive and negative
															   //acceleration
		if ((accelerationx[1] <= WINDOW) && (accelerationx[1] >= -WINDOW)) //Discrimination window applied
		{
			accelerationx[1] = 0;
		}
		if ((accelerationy[1] <= WINDOW) && (accelerationy[1] >= -WINDOW)) //Discrimination window applied
		{
			accelerationy[1] = 0;
		}
		if ((accelerationz[1] <= WINDOW) && (accelerationz[1] >= -WINDOW)) //Discrimination window applied
		{
			accelerationz[1] = 0;
		}
		accelerationx[1] = (short)((accelerationx[1]) * (REDUCTION_FACTOR)); //eliminating zero reference
																//offset of the acceleration data
		accelerationy[1] = (short)((accelerationy[1]) * (REDUCTION_FACTOR)); // to obtain positive and negative
															   //acceleration
		accelerationz[1] = (short)((accelerationz[1]) * (REDUCTION_FACTOR)); // to obtain positive and negative
#ifdef DEBUG
		printf("\n Accelerationx = %i",accelerationx[1]);
		printf("\n Accelerationy = %i",accelerationy[1]);
		printf("\n Accelerationz = %i",accelerationz[1]);
#endif
		//first X integration:
		velocityx[1]= velocityx[0]+ accelerationx[0]+ (((accelerationx[1] -accelerationx[0])/2) * DelT);
		//second X integration:
		positionX[1]= positionX[0] + velocityx[0] + (((velocityx[1] - velocityx[0])/2) * DelT);
		//first Y integration:
		velocityy[1] = velocityy[0] + accelerationy[0] + (((accelerationy[1] -accelerationy[0])/2) * DelT);
		//second Y integration:
		positionY[1] = positionY[0] + velocityy[0] + (((velocityy[1] - velocityy[0])/2) * DelT);
		//first Z integration:
		velocityz[1] = velocityz[0] + accelerationz[0] + (((accelerationz[1] -accelerationz[0])/2) * DelT);
		//second Y integration:
		positionZ[1] = positionZ[0] + velocityz[0] + (((velocityz[1] - velocityz[0])/2) * DelT);

		accelerationx[0] = accelerationx[1];
		//to the previous acceleration
		accelerationy[0] = accelerationy[1];
		//acceleration value.
		accelerationz[0] = accelerationz[1];
		//acceleration value.
		//The current acceleration value must be sent
		//variable in order to introduce the new
		velocityx[0] = velocityx[1];
		velocityy[0] = velocityy[1];
		velocityz[0] = velocityz[1]; //Same done for the velocity variable

		/* movement_end_check */
		if (accelerationx[1]==0)
		{ 
			countx++;
		}
		else 
		{ 
			countx = 0;
		}
		if (countx >= NO_ACCELERATION_COUNT)
		{
			velocityx[1]=0;
			velocityx[0]=0;
		}
		//we count the number of acceleration samples that equals cero
		//if this number exceeds 25, we can assume that velocity is cero
		if (accelerationy[1] == 0)
		{ 
			county++;
		}
		else
		{ 
			county =0;
		}
		//we do the same for the Y axis
		if (county >= NO_ACCELERATION_COUNT)
		{
			velocityy[1]=0;
			velocityy[0]=0;
		}
		//we count the number of acceleration samples that equals cero
		//if this number exceeds 25, we can assume that velocity is cero
		if (accelerationz[1] == 0)
		{ 
			countz++;
		}
		else
		{ 
			countz =0;
		}
		//we do the same for the Y axis
		if (countz >= NO_ACCELERATION_COUNT)
		{
			velocityz[1]=0;
			velocityz[0]=0;
		}
		
#ifdef DEBUG
		printf("\nDeltaT = %f \n positionX = %i\n positionY = %i\n positionZ = %i\n",DelT,positionX[1],positionY[1],positionZ[1]);
#endif
		positionX[0] = positionX[1];
		positionY[0] = positionY[1];
		positionZ[0] = positionZ[1];

		Gyro_X_Rate[1] = (float)((Gyro_X_sum - (int)Gyro_X_Offset) /  gyro_xsensitivity);
		Gyro_Y_Rate[1] = (float)((Gyro_Y_sum - (int)Gyro_Y_Offset) /  gyro_ysensitivity);
		Gyro_Z_Rate[1] = (float)((Gyro_Z_sum - (int)Gyro_Z_Offset) /  gyro_zsensitivity);
#ifdef DEBUG
        printf("Gyro_X_Rate[1] = %f ,Gyro_X_sum = %d ,Gyro_X_Offset = %d gyro_xsensitivity = %f)",Gyro_X_Rate[1],Gyro_X_sum , Gyro_X_Offset, gyro_xsensitivity);
		printf("\nGyro_X_Rate(GYRO) = %f\n",Gyro_X_Rate[1]);
		printf("Gyro_Y_Rate(GYRO) = %f\n",Gyro_Y_Rate[1]);
		printf("Gyro_Z_Rate(GYRO) = %f\n",Gyro_Z_Rate[1]);
#endif
		/* Single integration to Angular rate to angle */
		Gyro_X_Angle[1] = Gyro_X_Angle[0] + Gyro_X_Rate[0] + (((Gyro_X_Rate[1] - Gyro_X_Rate[0])/2 )* DelT);
		Gyro_Y_Angle[1] = Gyro_Y_Angle[0] + Gyro_Y_Rate[0] + (((Gyro_Y_Rate[1] - Gyro_Y_Rate[0])/2 )* DelT);
		Gyro_Z_Angle[1] = Gyro_Z_Angle[0] + Gyro_Z_Rate[0] + (((Gyro_Z_Rate[1] - Gyro_Z_Rate[0])/2 )* DelT);

#ifdef DEBUG
		printf("\nAngleX(GYRO) = %f\n",Gyro_X_Angle[1]);
		printf("AngleY(GYRO) = %f\n",Gyro_Y_Angle[1]);
		printf("AngleZ(GYRO) = %f\n",Gyro_Z_Angle[1]);
#endif
		/* Call kalman filter here */
		Kalman_X_Angle = kalman_updateX(Gyro_X_Angle[1],(float)accelerationy[1],(float)accelerationz[1],DelT);
		Kalman_Y_Angle = kalman_updateY(Gyro_Y_Angle[1],(float)accelerationx[1],(float)accelerationz[1],DelT);
		Kalman_Z_Angle = Gyro_Z_Angle[1];
#ifdef DEBUG
		printf("\n KalmanX(GYRO) = %f\n",Kalman_X_Angle);
		printf("KalmanY(GYRO) = %f\n",Kalman_Y_Angle);
		printf("KalmanZ(GYRO) = %f\n",Kalman_Z_Angle);
#endif
		Gyro_X_Angle[0] = Gyro_X_Angle[1];
		Gyro_Y_Angle[0] = Gyro_Y_Angle[1];
		Gyro_Z_Angle[0] = Gyro_Z_Angle[1];
		Gyro_X_Rate[0] = Gyro_X_Rate[1];
		Gyro_Y_Rate[0] = Gyro_Y_Rate[1];
		Gyro_Z_Rate[0] = Gyro_Z_Rate[1];

		/* Message sending to TCP part */
		Gyro_X_Angle_L = Pre_Gyro_Xangle;
		Gyro_Y_Angle_L = Pre_Gyro_Yangle;
		Gyro_Z_Angle_L = Pre_Gyro_Zangle;
		if (filtering_type == ANGULAR_INTEGRATION)
		{
			if(!isnanf(Gyro_X_Angle[1]))
			{
				Gyro_X_Angle_L = Gyro_X_Angle[1];
			}
			if(!isnanf(Gyro_Y_Angle[1]))
			{
				Gyro_Y_Angle_L = Gyro_Y_Angle[1];
			}
			if(!isnanf(Gyro_Z_Angle[1]))
			{
				Gyro_Z_Angle_L = Gyro_Z_Angle[1];
			}
	    }
	    else
	    {
			if(!isnanf(Kalman_X_Angle))
			{
				Gyro_X_Angle_L = Kalman_X_Angle;
			}
			if(!isnanf(Kalman_Y_Angle))
			{
				Gyro_Y_Angle_L = Kalman_Y_Angle;
			}
			if(!isnanf(Kalman_Z_Angle))
			{
				Gyro_Z_Angle_L = Kalman_Z_Angle;
			}
	    }
		sprintf(sendline,"%f %f %f %f %f %f\n",
											   (float)((positionY[0] - PrePositionY) / POSITION_CALIBRATION),
											   (float)((positionZ[0] - PrePositionZ) / POSITION_CALIBRATION),
											   (float)((positionX[0] - PrePositionX) / POSITION_CALIBRATION),
											   (float)((Gyro_Y_Angle_L - Pre_Gyro_Yangle) / ANGLE_CALIBRATION),
											   (float)((Gyro_Z_Angle_L - Pre_Gyro_Zangle) / ANGLE_CALIBRATION),
											   (float)((Gyro_X_Angle_L - Pre_Gyro_Xangle) / ANGLE_CALIBRATION)
											   );
		if(sendto(sockfd,sendline,strlen(sendline),0,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0)
		{
			printf("TCP : error \n");
		}
		else
		{
			PrePositionX = positionX[0];
			PrePositionY = positionY[0];
			PrePositionZ = positionZ[0];

			Pre_Gyro_Xangle = Gyro_X_Angle_L;
			Pre_Gyro_Yangle = Gyro_Y_Angle_L;
			Pre_Gyro_Zangle = Gyro_Z_Angle_L;
#ifdef DEBUG
			printf("\n ANGLE X = %f\n",Pre_Gyro_Xangle);
			printf("\n ANGLE Y = %f\n",Pre_Gyro_Yangle);
			printf("\n ANGLE Z = %f\n",Pre_Gyro_Zangle);
#endif
		}
//#ifdef DEBUG
		printf("TCP has sent\n");
		printf(sendline);
//#endif
	    Accel_X_sum = 0;
	    Accel_Y_sum = 0;
	    Accel_Z_sum = 0;
	    Gyro_X_sum = 0;
	    Gyro_Y_sum = 0;
	    Gyro_Z_sum = 0;
		/* Check if reset position is required */
		if(position_reset)
		{
			positionX[0] = 0;
			positionX[1] = 0;
			positionY[0] = 0;
			positionY[1] = 0;
			positionZ[0] = 0;
			positionZ[1] = 0;
			velocityx[0] = 0;
			velocityx[1] = 0;
			velocityy[0] = 0;
			velocityy[1] = 0;
			velocityz[0] = 0;
			velocityz[1] = 0;
			accelerationx[0] = 0;
			accelerationx[1] = 0;
			accelerationy[0] = 0;
			accelerationy[1] = 0;
			accelerationz[0] = 0;
			accelerationz[1] = 0;
			Gyro_X_Angle[1] = 0;
			Gyro_Y_Angle[1] = 0;
			Gyro_Z_Angle[1] = 0;
			Gyro_X_Rate[1] = 0;
			Gyro_Y_Rate[1] = 0;
			Gyro_Z_Rate[1] = 0;
			Gyro_X_Angle[0] = 0;
			Gyro_Y_Angle[0] = 0;
			Gyro_Z_Angle[0] = 0;
			Gyro_X_Rate[0] = 0;
			Gyro_Y_Rate[0] = 0;
			Gyro_Z_Rate[0] = 0;
			Pre_Gyro_Xangle = 0;
			Pre_Gyro_Yangle = 0;
			Pre_Gyro_Zangle = 0;
			Gyro_X_Angle_L = 0;
			Gyro_Y_Angle_L = 0;
			Gyro_Z_Angle_L = 0;
			if(ButtonPressed == LEFT_BUTTON)
			{
				if (filtering_type == KALMAN_FILTERING)
				{
					/* make the filtering type ANGULAR velocity integration type */
					filtering_type = ANGULAR_INTEGRATION;
				}
			}
			else
			{
				if (filtering_type == ANGULAR_INTEGRATION)
				{
					/* make the filtering type kalman type */
					filtering_type = KALMAN_FILTERING;
				}
			}
			/* Atomically Reset the flag */
			__sync_sub_and_fetch(&position_reset,1);
			sleep(1);
		}
	}
	return NULL;
}
/* *********************************************************************
 * NAME:             MouseHandlerThread
 * CALLED BY:        Started by main() thread
 * DESCRIPTION:      Handles the button click from the mouse
 * INPUT PARAMETERS: dummy not used
 * RETURN VALUES:    None
 ***********************************************************************/
void* MouseHandlerThread(void* dummy)
{
	int MouseEventFd;
	struct input_event MouseEvent;
	if((MouseEventFd = open("/dev/event2", O_RDONLY)) < 0)
	{
		perror("opening device");
		return NULL;
	}

	while(read(MouseEventFd, &MouseEvent, sizeof(struct input_event)))
	{
		if((MouseEvent.type == EV_KEY) && (MouseEvent.value == 0))
		{
			if(MouseEvent.code == BTN_LEFT)
			{
				printf("\n Left Button was released  \n the computation of orientation follows angular velocity");
				if(ButtonPressed == RIGHT_BUTTON)
				{
					/* atomic subtraction */
					 __sync_sub_and_fetch(&ButtonPressed,1);
				}
			}	
			else
			{
				if(ButtonPressed == LEFT_BUTTON)
				{
					/* atomic subtraction */
					 __sync_add_and_fetch(&ButtonPressed,1);
				}
				printf("\n Someother Button was released \n the computation of orientation follows kallman filter");
			}
			if((MouseEvent.code == BTN_LEFT) || (MouseEvent.code == BTN_RIGHT))
			{
				printf("\nresetting the position to (0,0,0)");
				/* atomic instruction */
		        __sync_add_and_fetch(&position_reset,1);
			}	
		}
		usleep(1000);
	}
	printf("\n Exiting the MouseHandlerThread");
}
/* *********************************************************************
 * NAME:             DataAcquisitionThread
 * CALLED BY:        Started by main() thread
 * DESCRIPTION:      Interrupt handler for data ready signal
 * INPUT PARAMETERS: dummy not used
 * RETURN VALUES:    None
 ***********************************************************************/
void* DataAcquisitionThread(void* dummy)
{
	int FdEch,FdEchV,res,test;
	struct pollfd PollEch = {0};
	unsigned long long StartTime, StopTime;
	unsigned char ReadValue[4];
	/* Open the edge and value files */
	FdEch = open("/sys/class/gpio/gpio15/edge", O_WRONLY);
	if (FdEch < 0)
	{
		printf("\n gpio15 edge open failed");
	}
	FdEchV = open("/sys/class/gpio/gpio15/value", O_RDONLY|O_NONBLOCK);
	if (FdEchV < 0)
	{
		printf("\n gpio15 value open failed");
	}
    /* Prepare poll fd structure */
    PollEch.fd = FdEchV;
    PollEch.events = POLLPRI|POLLERR;
    lseek(FdEchV, 0, SEEK_SET);
	res = pread(FdEchV,&ReadValue,sizeof(ReadValue),0);
#ifdef DEBUG
	printf("\n ResInt = %i",res);
    printf("\nRead out");
#endif
	/* Change the edge trigger to rising edge */
	write(FdEch,"rising",6);

    do
    {
		lseek(FdEchV, 0, SEEK_SET);
		/* Start polling for the rising edge now */
#ifdef DEBUG
		printf("\n Waiting for Interrupt to arrive\n");
#endif
		poll(&PollEch,1,/* 1000 */ -1);
		if (PollEch.revents & POLLPRI)
		{
			/* Check if the thread is ready to process the the data */
			if(DATA_PROCESSED == DataReady)
			{
				/* Process thread is ready to receive new data */
			   /* Clear the value file so that  it can receive a new interrupt */
			   res = pread(FdEchV,&ReadValue,sizeof(ReadValue),0);
#ifdef DEBUG
			   printf("\nPread = %d",res);
			   printf("\n DataAcquisitionThread : calling __sync_add_and_fetch\n");
#endif
			   test = __sync_add_and_fetch(&DataReady,1);
#ifdef DEBUG
			   printf("\n DataAcquisitionThread : Result from  __sync_add_and_fetch, test = %i \n",test);
#endif
               sys_futex((void*)&DataReady,FUTEX_WAKE,DATA_READY,NULL,NULL,0);
#ifdef DEBUG
			   printf("\n Sys_futex in DataAcquisitionThread, DataReady = %d",DataReady);
#endif
			}
			else
			{
				/* New data arrived before the previous data was processed, hence print */
				printf("\nCongestion :  Rate of measurement is faster than the rate of processing\n");
			}
		}
		else
		{
			printf("\nError detecting rising edge");
		}
    }
    while(1);
    /*Run till the timout flag is set by the main thread */
	printf("\n Ending Distance measurement");
	close(FdEch);
	return NULL;
}
/* *********************************************************************
 * NAME:             CalibrationThread
 * CALLED BY:        Started by main() thread
 * DESCRIPTION:      Read data from the sensor and takes the average to
 *                   calibrate the device
 * INPUT PARAMETERS: dummy not used
 * RETURN VALUES:    None
 ***********************************************************************/
void* CalibrationThread(void* dummy)
{
	unsigned short counter = 0;
	unsigned char DataAcquired[14];
	int test;
	int Accel_X_sum = 0, Accel_Y_sum = 0, Accel_Z_sum = 0;
	short Accel_X_L = 0, Accel_Y_L = 0, Accel_Z_L = 0;
	int Gyro_X_sum = 0, Gyro_Y_sum = 0, Gyro_Z_sum = 0;
	short Gyro_X_L = 0, Gyro_Y_L = 0, Gyro_Z_L = 0;
	/* Do a dummy read to clear off the interrupt line */
	SensorRead(MPU6050_RA_ACCEL_XOUT_H,&DataAcquired[0],14);
	while(counter < CALIBRATION_COUNT)
	{
		counter++;
		/* go to sleep till the data ready interrupt arrive */
#ifdef DEBUG
		printf("\n CalibrationThread going to sleep at futex, DataReady = %i \n",DataReady);
#endif
		sys_futex((void*)&DataReady,FUTEX_WAIT,DATA_PROCESSED,NULL,NULL,0);
#ifdef DEBUG
		printf("\n CalibrationThread woken up from sleep after futex returned, DataReady = %i \n",DataReady);
#endif
		/* read 14 bytes data having gyro , acceleration, temperature */
		SensorRead(MPU6050_RA_ACCEL_XOUT_H,&DataAcquired[0],14);
#ifdef DEBUG
		printf("\n At CalibrationThread , before sub_and_fetch , Test = %i, DataReady = %i ",test,DataReady);
#endif
		/* Make the Data ready  variable processed */
		test = __sync_sub_and_fetch(&DataReady,1);
#ifdef DEBUG
		printf("\n At CalibrationThread , after sub_and_fetch , Test = %i, DataReady = %i ",test,DataReady);
#endif
		/* decode the data */
		Accel_X_L = ((DataAcquired[0]<<8)|DataAcquired[1]);
		Accel_Y_L = ((DataAcquired[2]<<8)|DataAcquired[3]);
		Accel_Z_L = ((DataAcquired[4]<<8)|DataAcquired[5]);
		Accel_X_sum = Accel_X_sum + (int)Accel_X_L;
		Accel_Y_sum = Accel_Y_sum + (int)Accel_Y_L;
		Accel_Z_sum = Accel_Z_sum + (int)Accel_Z_L;
		Gyro_X_L = ((DataAcquired[8]<<8)|DataAcquired[9]);
		Gyro_Y_L = ((DataAcquired[10]<<8)|DataAcquired[11]);
		Gyro_Z_L = ((DataAcquired[12]<<8)|DataAcquired[13]);
		Gyro_X_sum = Gyro_X_sum + (int)(Gyro_X_L);
		Gyro_Y_sum = Gyro_Y_sum + (int)(Gyro_Y_L);
		Gyro_Z_sum = Gyro_Z_sum + (int)(Gyro_Z_L);
	}

	Accel_X_Offset = (short)(Accel_X_sum / CALIBRATION_COUNT);
	Accel_Y_Offset = (short)(Accel_Y_sum / CALIBRATION_COUNT);
	Accel_Z_Offset = (short)(Accel_Z_sum / CALIBRATION_COUNT);

	Gyro_X_Offset = (short)(Gyro_X_sum / CALIBRATION_COUNT);
	Gyro_Y_Offset = (short)(Gyro_Y_sum / CALIBRATION_COUNT);
	Gyro_Z_Offset = (short)(Gyro_Z_sum / CALIBRATION_COUNT);

	printf("\n Accel_X_Cal = %i",Accel_X_Offset);
	printf("\n Accel_Y_Cal = %i",Accel_Y_Offset);
	printf("\n Accel_Z_Cal = %i",Accel_Z_Offset);

	printf("\n Gyro_X_Cal = %i",Gyro_X_Offset);
	printf("\n Gyro_Y_Cal = %i",Gyro_Y_Offset);
	printf("\n Gyro_Z_Cal = %i",Gyro_Z_Offset);
}
/* *********************************************************************
 * NAME:             main
 * CALLED BY:        main thread
 * DESCRIPTION:      Overall execution of the program
 * INPUT PARAMETERS: none
 * RETURN VALUES:    None
 ***********************************************************************/
main()
{
	int error = 1;
	pthread_t MouseHandlerThreadId, DataAcquisitionThreadId,CalibrationThreadId,DataProcessingThreadId;
	SensorInit();
	do
	{
		Setup_MPU6050();
		MPU6050_Test_I2C();
		error = MPU6050_Check_Registers();
		printf("Setting up MPU6050 \n");
	}
	while(error==1);

    printf("\n Calibrating Accelerometer and Gyros \n");
    pthread_create(&CalibrationThreadId,NULL,&CalibrationThread,NULL);
    pthread_create(&DataAcquisitionThreadId,NULL,&DataAcquisitionThread,NULL);  
    /* Wait for Calibration to get over */
    pthread_join(CalibrationThreadId, NULL);  
    printf("\n Finished : Calibration of Accelerometer and Gyros \n");
	/* create the data processing thread */
    pthread_create(&DataProcessingThreadId,NULL,&DataProcessingThread,NULL);
    /* create the mouse click handler thread that checks every second for mouse clicks */
    pthread_create(&MouseHandlerThreadId,NULL,&MouseHandlerThread,NULL);
    /* wait for data processing thread to exit, however user should terminate the entire program with cntrl + C */
    pthread_join(DataProcessingThreadId, NULL);
	printf(" Main function ended");
}

