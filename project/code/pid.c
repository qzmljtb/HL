#include "pid.h"                                      //PID控制模块头文件

//俯仰环(Pitch)PID实例
PID_IncTypeDef   pitch_angle_wheel, pitch_anglespeed_wheel, pitch_speed_wheel;
float            pidout_1 = 0, pidout_2 = 0, pidout_3 = 0;   //俯仰三环PID输出

//偏航环(Yaw)PID实例
PID_IncTypeDef   yaw_angle_wheel, yaw_anglespeed_wheel;
float            pidout_4, pidout_5;                          //偏航双环PID输出

//横滚环(Roll)PID实例——腿部左右平衡
PID_IncTypeDef   roll_angle_leg;
float            pidout_roll_leg = 0;                         //横滚PID输出(腿部左右差动量)


float pitch_zero = -8.0f;                              //俯仰机械零点(° 小车静止平衡角度)
float roll_zero  = 0.0f;                              //横滚机械零点(° 左右平衡角度)
float run_speed = 0;                                   //目标行驶速度
float Angle_Z = 0;                                     //目标偏航角(°)

//PID结构体初始化
void PID_Inc_Init(PID_IncTypeDef* sptr, float kp, float ki, float kd, float max_i, float max_out)
{
    sptr->Ek = 0;               //本次误差
    sptr->Ek1 = 0;              //上次误差
    sptr->Ek2 = 0;              //上上次误差/积分累加
    sptr->Kp = kp;              //比例系数
    sptr->Ki = ki;              //积分系数
    sptr->Kd = kd;              //微分系数
    sptr->max_i = max_i;        //积分限幅
    sptr->max_out = max_out;    //输出限幅
}

//PID参数配置(所有控制环参数集中设置)
void PID_param_init(void)
{
    /*
     *  串级PID结构:
     *  俯仰(Pitch)三环: 速度环(外) → 角度环(中) → 角速度环(内)
     *  偏航(Yaw)双环  : 角度环(外) → 角速度环(内)
     */
  
    // === 俯仰环(Pitch) ===
    //速度环 (控制小车前后移动速度)       P项      I项     D项  积分限幅 输出限幅
    PID_Inc_Init(&pitch_speed_wheel,        6.5f,   0.0f,   6.5f,   500,   2000);
    //角度环 (控制车身俯仰角=平衡核心) 
    PID_Inc_Init(&pitch_angle_wheel,        800.0f, 0.15f,  0.0f,   400,   8000);
    //角速度环 (控制俯仰角速度=内环阻尼) 
    PID_Inc_Init(&pitch_anglespeed_wheel,   1.5f,   0.07f,  0.02f,  800,   8000);
    
    // === 偏航环(Yaw) ===
    //角度环 (控制偏航角=转向目标)
    PID_Inc_Init(&yaw_angle_wheel,          200.0f, 0,      350,    1000,  4000);
    //角速度环 (控制偏航角速度=内环阻尼)
    PID_Inc_Init(&yaw_anglespeed_wheel,     1.5f,   0.0f,   0,      1000,  3000);

    // === 横滚环(Roll 腿部左右平衡 单边桥保持) ===
    //角度环 (控制横滚角=左右平衡 输出→腿部左右差动)
    //方向已确认:左倾→左腿伸长✓ Kp+Kd快速响应 Ki消除稳态误差
    PID_Inc_Init(&roll_angle_leg,           400.0f, 0.05f,  60.0f,  200,   2000);
}


//位置式PID(实际为增量式+积分累加)
float Positional_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue)
{
    float PIDInc;                                            //PID计算输出
    PID->Ek1 = PID->Ek;                                      //保存上次误差
    PID->Ek = SetValue - ActualValue;                        //计算本次误差
    PID->Ek2 += PID->Ek;                                     //积分累加(Ek2用作积分项)
    
    //积分限幅(防止积分饱和)
    if (PID->Ek2 > PID->max_i)  PID->Ek2 = PID->max_i;
    if (PID->Ek2 < -PID->max_i) PID->Ek2 = -PID->max_i;
    
    //PID公式: P*Ek + I*∑Ek + D*(Ek-Ek1)
    PIDInc =(PID->Kp * PID->Ek) +
            (PID->Ki * PID->Ek2)+
            (PID->Kd * (PID->Ek - PID->Ek1));
    
    //输出限幅
    if (PIDInc > PID->max_out)  PIDInc = PID->max_out;
    if (PIDInc < -PID->max_out) PIDInc = -PID->max_out;

    return PIDInc;
}

//增量式PID(严格增量形式 输出=上次输出+增量)
float Incremental_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue)
{
    static float PIDInc = 0;                                 //PID输出(静态保持上次值)
    PID->Ek2 = PID->Ek1;                                     //上上次←上次
    PID->Ek1 = PID->Ek;                                      //上次←本次
    PID->Ek = SetValue - ActualValue;                        //计算本次误差

    //积分限幅
    if (PID->Ek2 > PID->max_i)  PID->Ek2 = PID->max_i;
    if (PID->Ek2 < -PID->max_i) PID->Ek2 = -PID->max_i;

    //增量公式: Δu = P*(Ek-Ek1) + I*Ek + D*(Ek-2*Ek1+Ek2)
    PIDInc +=(PID->Kp * (PID->Ek - PID->Ek1)) +
             (PID->Ki * PID->Ek)+
             (PID->Kd * (PID->Ek - 2*PID->Ek1 + PID->Ek2));
    
    //输出限幅
    if (PIDInc > PID->max_out)  PIDInc = PID->max_out;
    if (PIDInc < -PID->max_out) PIDInc = -PID->max_out;

    return PIDInc;
}


// ==================== 俯仰环(Pitch)串级 ====================

//俯仰角速度环(最内环 陀螺仪X轴角速度反馈)
float PITCH_ANGLESPEED_LOOP(float value)
{
    float SetValue = value;                                  //期望角速度
    float ActualValue = -imu660rb_gyro_y;                     //实际角速度(陀螺仪Y轴)
    float out = Positional_PID(&pitch_anglespeed_wheel, SetValue, ActualValue);
    return out;
}

//俯仰角度环(中间环 欧拉角Pitch反馈)
float PITCH_ANGLE_LOOP(float value)
{
    float SetValue = value;                                  //期望俯仰角
    float ActualValue = (euler_angle.pitch - pitch_zero);    //实际俯仰角(减去机械零点)
    float out = Positional_PID(&pitch_angle_wheel, SetValue, ActualValue);
    return out;
}

//速度环(最外环 电机编码器速度反馈)
float PITCH_SPEED_LOOP(float value)
{
    float SetValue = value;                                  //期望速度
    float actual_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;  //左右轮平均速度
    float ActualValue = -run_speed - actual_speed;           //误差=(目标速度-实际速度)取反
    float out = Positional_PID(&pitch_speed_wheel, SetValue, ActualValue);
    return out;
}

// ==================== 偏航环(Yaw)串级 ====================

//偏航角速度环(内环 陀螺仪Z轴角速度反馈)
float YAW_ANGLESPEED_LOOP(float value)
{
    float SetValue = value;                                  //期望偏航角速度
    float ActualValue = -imu660rb_gyro_z;                     //实际角速度(陀螺仪Z轴)
    float out = Positional_PID(&yaw_anglespeed_wheel, SetValue, ActualValue);
    return out;
}

//偏航角度环(外环 欧拉角Yaw反馈 含±180°跳变处理)
float YAW_ANGLE_LOOP(float value)
{
    float SetValue = value;                                  //期望偏航角
    float ActualValue = 0;                                   //实际偏航角偏差
    //处理角度±180°跳变问题: 当跨越±180°边界时修正偏差计算
    if (euler_angle.yaw <= -120 && Angle_Z >= 120)
        ActualValue = euler_angle.yaw - (Angle_Z - 360);     //当前角-120°, 目标角+120°→从+180°绕到-180°
    else if (euler_angle.yaw >= 120 && Angle_Z <= -120)
        ActualValue = euler_angle.yaw - (360 + Angle_Z);     //当前角+120°, 目标角-120°→从-180°绕到+180°
    else
        ActualValue = euler_angle.yaw - Angle_Z;             //正常范围直接作差
    float out = Positional_PID(&yaw_angle_wheel, SetValue, ActualValue);
    return out;
}

// ==================== 横滚环(Roll 腿部左右平衡) ====================

//横滚角度环(欧拉角Roll反馈→腿部左右差动)
float ROLL_ANGLE_LEG_LOOP(float value)
{
    float SetValue = value;                                  //期望横滚角(通常为0=水平)
    float ActualValue = (euler_angle.roll - roll_zero);     //实际横滚角(减去机械零点)
    float out = Positional_PID(&roll_angle_leg, SetValue, ActualValue);
    return out;
}