
/* Header file included by kalmanX.c and kalmanY.c to make global variable available to main() */
/* kalman.c was modified from the source : A quadrocoper control example which includes MPU 6050 and Kalman filer code.
 * https://github.com/big5824/Quadrocopter(see the description at http://www.botched.co.uk/pic-tutorials/mpu6050-setup-data-aquisition/)
 */
extern float angle_x;
extern float q_bias_x;
extern float rate_x;

extern float angle_y;
extern float q_bias_y;
extern float rate_y;

extern float kalman_updateX(const float q_m,const float ax_m, const float az_m,const float dt);
extern float kalman_updateY(const float q_m,const float ax_m, const float az_m,const float dt);
