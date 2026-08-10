#include "zf_common_headfile.h"                       //逐飞库通用头文件

#ifndef CODE_PID_H_
#define CODE_PID_H_




//PID增量式结构体(位置式也复用此结构 Ek2在位置式中用作积分累加)
typedef struct
{
    float Kp;                       //比例常数 P项
    float Ki;                       //积分常数 I项
    float Kd;                       //微分常数 D项
    float Ek;                       //本次误差
    float Ek1;                      //上次误差
    float Ek2;                      //上上次误差(增量式) / 积分累加(位置式)
    float max_i;                    //积分限幅
    float max_out;                  //输出限幅
} PID_IncTypeDef;

// === 全局声明 ===

//俯仰环(Pitch)三环PID实例
extern PID_IncTypeDef   pitch_angle_wheel, pitch_anglespeed_wheel, pitch_speed_wheel;
extern float            pidout_1, pidout_2, pidout_3;       //俯仰三环PID输出值

//偏航环(Yaw)双环PID实例
extern PID_IncTypeDef   yaw_angle_wheel, yaw_anglespeed_wheel;
extern float            pidout_4, pidout_5;                  //偏航双环PID输出值

//横滚环(Roll)单环PID实例——腿部左右平衡
extern PID_IncTypeDef   roll_angle_leg;                      //横滚角度环(控制腿部伸缩)
extern float            pidout_roll_leg;                     //横滚PID输出(腿部左右差动量)

//全局控制变量
extern float pitch_zero;                                     //俯仰机械零点(°)
extern float roll_zero;                                      //横滚机械零点(°)
extern float run_speed;                                      //目标行驶速度
extern float Angle_Z;                                        //目标偏航角(°)

// === 函数声明 ===

//PID初始化
void PID_Inc_Init(PID_IncTypeDef* sptr, float kp, float ki, float kd, float max_i, float max_out);
//PID参数集中配置
void PID_param_init(void);

//PID算法核心
float Positional_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue);    //位置式PID(Ek2作积分累加)
float Incremental_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue);   //增量式PID(标准增量形式)

//俯仰环(Pitch)三环——平衡控制
float PITCH_ANGLE_LOOP(float value);                         //角度环(中环 欧拉角Pitch)
float PITCH_ANGLESPEED_LOOP(float value);                    //角速度环(内环 陀螺仪X)
float PITCH_SPEED_LOOP(float value);                         //速度环(外环 编码器)

//偏航环(Yaw)双环——转向控制
float YAW_ANGLESPEED_LOOP(float value);                      //角速度环(内环 陀螺仪Z)
float YAW_ANGLE_LOOP(float value);                           //角度环(外环 欧拉角Yaw)

//横滚环(Roll)单环——腿部左右平衡
float ROLL_ANGLE_LEG_LOOP(float value);                      //横滚角度环(欧拉角Roll→腿部差动)

#endif