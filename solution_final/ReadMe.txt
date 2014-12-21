/*******************************************************************************************************************
********************************** Gyroscope and accelerometer sensing and filter **********************************
********************************************************************************************************************/

1) To compile the program please run this command:
$CC main_4.c MPU6050.c kalmanX.c kalmanY.c -o main_4 -lm -lpthread

2) The I2C slave address is set as 0x69, it can be changed by updating the macro #define I2C_SENSOR_DEVICE_ADDRESS

3) Unity Proj should be configured to have the IP address "169.254.147.71" and port number 9999, if you want to change the 
SW to connect to different IP, change the macros #define IP_ADDRESS  #define PORT_NUMBER	

4) Please use the first version of UnityProj program for testing.

5)To find the highest sampling rate I changed the sampling division register with values 0x30,0x16,0x17,0x18,0x19 and 0x1A.
I took the log for each run. And counted number of time the "Conjestion" vs "DetT(success)". Since the average of 5 values 
are done in the program, we need to divide the DelT by 5. The following table summarizes the network conjestion for sensor read


-------------------------------------------------------------------------------------------------------------------
--sl------ Sam Div Register -------------------No. of Conjestion------------------------Number of success----------
-------------------------------------------------------------------------------------------------------------------
  1.          0x16 									8742									2935
  2.		  0x17									9103									4271
  3.		  0x18									3283									4989
  4.		  0x19									1062									4595
  5.		  0x30									0										3541
  
  
observing the above table, we can see that transfer rate for 0x19 is highest. It has DelT = 0.014574 seconds for transfering 
5 datasets from the sensor. Hence it takes 0.014574/5 = 2.9148ms . for one data set transfer.

               8000
Frequence =-------------- = 307Hz or 307 samples per seconds
             1 + 25(dec)     									
