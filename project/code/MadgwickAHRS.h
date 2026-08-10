//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include "zf_common_headfile.h"
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
typedef struct
{
    float P; //估算协方差
    float G; //卡尔曼增益
    float Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    float R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    float Output; //卡尔曼滤波器输出
}KFPTypeS; //Kalman Filter parameter type Struct

//-------------------------------------------------------------------------------------------
// Function declarations
void begin(float sampleFrequency);
void computeAngles(void);

void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
//float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
//float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
//float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);
void getangle(void);
float KalmanFilter(KFPTypeS *kfp, float input);
extern float Roll,Pitch,Yaw;
extern float icm_data_gyro_x,icm_data_gyro_y,icm_data_gyro_z,kalman_gyro_x;
#endif

