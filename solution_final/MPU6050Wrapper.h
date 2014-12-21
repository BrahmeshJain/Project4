/* Wrapper to use MPU6050.c and MPU6050.h from A quadrocoper control example which includes MPU 6050 and Kalman filer code.
 * https://github.com/big5824/Quadrocopter (see the description at http://www.botched.co.uk/pic-tutorials/mpu6050-setup-data-aquisition/)
 */
extern void SensorWrite(unsigned char RegAddress, unsigned char Value);
extern void SensorRead(unsigned char RegAddress, unsigned char* buff , unsigned short size);
#define LDByteWriteI2C(SlaveAddress,Reg,Value) \
  do \
  { \
	  SensorWrite(Reg,Value); \
  } \
  while(0)
  
#define LDByteReadI2C(SlaveAddress,Reg,buf,size) \
  do \
  { \
	  SensorRead(Reg,buf,size); \
  } \
  while(0)
