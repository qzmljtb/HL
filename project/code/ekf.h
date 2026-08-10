#include "zf_common_headfile.h"

#ifndef CODE_EKF_H_
#define CODE_EKF_H_


#define DEG_TO_RAD      (57.295779513082320876798154814105f)
#define dt              (0.002f)
#define K1               (1.0f)
#define GYRO_BIAS_CALIB_SAMPLES (1500U)


typedef struct
{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
}imu_t;





void EKF_Init(void);
void EKF_UpData(void);
uint8 EKF_GyroBiasCalibrate(void);

extern uint8 gyro_calib_ok;                                  //零飘校准完成:0=校准中 1=完成


#endif /* CODE_EKF_H_ */
