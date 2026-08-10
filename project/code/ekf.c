#include "zf_common_headfile.h"



matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle;
imu_t imu_data = {0, 0, 0, 0, 0, 0};
uint8 gyro_calib_ok = 0;                                     //陀螺仪零飘校准完成标志
matrix_type r_yz = 0.001f;

const matrix_type q[4][4] = {{0.005, 0, 0, 0}, {0, 0.005, 0, 0}, {0, 0, 0.005, 0}, {0, 0, 0, 0.005}};
const matrix_type r[3][3] = {{10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}};
const matrix_type p[4][4] = {{1000000, 0, 0, 0}, {0, 1000000, 0, 0}, {0, 0, 1000000, 0}, {0, 0, 0, 1000000}};
const matrix_type ekf[4] = {1, 0, 0, 0};

static matrix_t Q;
static matrix_t R;
static matrix_t P;


void EKF_Init(void)
{
	Matrix_From_Array(&exf_x, (const matrix_type*)ekf, 4, 1);
	Matrix_From_Array(&Q, (const matrix_type*)q, 4, 4);
	Matrix_From_Array(&R, (const matrix_type*)r, 3, 3);
	Matrix_From_Array(&P, (const matrix_type*)p, 4, 4);
}




static inline void quaternion_to_euler(void)
{
    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    euler_angle.roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * DEG_TO_RAD;   // roll
    euler_angle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * DEG_TO_RAD;                                    // pitch
    euler_angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * DEG_TO_RAD;    // yaw
}




static float imu660rb_acc_x_l = 0;
static float imu660rb_acc_y_l = 0;
static float imu660rb_acc_z_l = 0;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float gyro_bias_sum_x = 0.0f;
static float gyro_bias_sum_y = 0.0f;
static float gyro_bias_sum_z = 0.0f;
static uint16 gyro_bias_samples = 0;

uint8 EKF_GyroBiasCalibrate(void)
{
    if (gyro_bias_samples < GYRO_BIAS_CALIB_SAMPLES)
    {
        gyro_bias_sum_x += imu660rb_gyro_x;
        gyro_bias_sum_y += imu660rb_gyro_y;
        gyro_bias_sum_z += imu660rb_gyro_z;
        gyro_bias_samples++;

        if (gyro_bias_samples == GYRO_BIAS_CALIB_SAMPLES)
        {
            gyro_bias_x = gyro_bias_sum_x / GYRO_BIAS_CALIB_SAMPLES;
            gyro_bias_y = gyro_bias_sum_y / GYRO_BIAS_CALIB_SAMPLES;
            gyro_bias_z = gyro_bias_sum_z / GYRO_BIAS_CALIB_SAMPLES;
            gyro_calib_ok = 1;                             //校准完成标志
            BUZZER_flag = 1;                                //蜂鸣器短鸣提示
            EKF_Init();
        }
    }
    return gyro_bias_samples == GYRO_BIAS_CALIB_SAMPLES;
}

void imu_get_values(void)
{
    //imu660rb_get_gyro();  // 已在 getangle() 中读取
   // imu660rb_get_acc();   // 已在 getangle() 中读取


    imu_data.acc_x = K1 * imu660rb_acc_x + (1 - K1) * imu660rb_acc_x_l;
    imu_data.acc_y = -(K1 * imu660rb_acc_y + (1 - K1) * imu660rb_acc_y_l);
    imu_data.acc_z = -(K1 * imu660rb_acc_z + (1 - K1) * imu660rb_acc_z_l);
    imu660rb_acc_x_l = imu_data.acc_x;
    imu660rb_acc_y_l = imu_data.acc_y;
    imu660rb_acc_z_l = imu_data.acc_z;

    imu_data.gyro_x = (imu660rb_gyro_x - gyro_bias_x) * PI / 180 / 14.3f;
    imu_data.gyro_y = -(imu660rb_gyro_y - gyro_bias_y) * PI / 180 / 14.3f;
    imu_data.gyro_z = -(imu660rb_gyro_z - gyro_bias_z) * PI / 180 / 14.3f;
}





void EKF_UpData(void)
{

    float gx, gy, gz;
    imu_get_values();
    gx = imu_data.gyro_x;
    gy = imu_data.gyro_y;
    gz = imu_data.gyro_z;

    matrix_t Z;

    Matrix_Init(&Z, 3, 1);

    Z.data[0][0] = (matrix_type)imu_data.acc_x;
    Z.data[1][0] = (matrix_type)imu_data.acc_y;
    Z.data[2][0] = (matrix_type)imu_data.acc_z;

    normalize_vector(&Z);

    matrix_type f[4][4]= {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                          {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                          {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                          {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};

    matrix_t F,FT;
    Matrix_From_Array(&F, (const matrix_type*)f, 4, 4);
    FT = Matrix_Transpose(&F);

    exf_x = multiply_matrices(&F, &exf_x);  //X = F * X;
    normalize_vector(&exf_x);

    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    matrix_type h[3][4]={{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
					     {2 * q1, 2 * q0, 2 * q3, 2 * q2},
					     {2 * q0, -2 * q1, -2 * q2, 2 * q3}};

    matrix_t H, HT;
    Matrix_From_Array(&H, (const matrix_type*)h, 3, 4);
    HT = Matrix_Transpose(&H);
    matrix_t PK_;

    //PK_ = F * P(K - 1) * FT + Q;
    PK_ = multiply_matrices(&F, &P);       //F * P;
    PK_ = multiply_matrices(&PK_, &FT);    //F * P * FT;
    P = add_matrices(&PK_, &Q);            //F * P * FT + Q;


    //DK_ = H * PK_ * HT + R;
    matrix_t DK, invDK;
    DK = multiply_matrices(&H, &P);
    DK = multiply_matrices(&DK, &HT);
    DK = add_matrices(&DK, &R);

    if(inverse_matrix(&DK, &invDK))
    {
    	quaternion_to_euler();
    	return;
    }

    //ek = Z - H * X;
    matrix_t EK, EKT;
    EK = multiply_matrices(&H, &exf_x);     //H * X;
    EK = subtract_matrices(&Z, &EK);        //Z - HX;
    EKT = Matrix_Transpose(&EK);

    //r = EKT * invDK * EK;
    error = multiply_matrices(&EKT, &invDK);
    error = multiply_matrices(&error, &EK);

    if(error.data[0][0] > r_yz)
    {
    	quaternion_to_euler();
    	return;
    }

    //Kk = M * P * HT * invDK;
    matrix_t Kk;
    Kk = multiply_matrices(&P, &HT);
    Kk = multiply_matrices(&Kk, &invDK);

    //X = X_ + Kk * Ek;
    matrix_t temp;
    temp = multiply_matrices(&Kk, &EK);
    exf_x = add_matrices(&exf_x, &temp);
    normalize_vector(&exf_x);

    //P = (I - Kk * H) * PK_;
    matrix_t I;
    Matrix_Identity(&I, 4);
    temp = multiply_matrices(&Kk, &H);
    temp = subtract_matrices(&I, &temp);
    P = multiply_matrices(&temp, &P);
	quaternion_to_euler();
	//printf("%f,%f,%f\n",dt,euler_angle.pitch,euler_angle.roll);
}
