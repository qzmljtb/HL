#include "zf_common_headfile.h"                       //逐飞库通用头文件

#ifndef _STEER_CONTROL_H_
#define _STEER_CONTROL_H_

// ===== 舵机1(左前 LF) =====
#define STEER_1_PWM     (TCPWM_CH30_P10_2)            //控制引脚 ATOM0_CH0_P21_2
#define STEER_1_FRE     (300             )            //PWM频率(Hz)
#define STEER_1_DIR     (1               )            //旋转方向(+1:车身高方向)
#define STEER_1_CENTER  (4500            )            //中位PWM值

// ===== 舵机2(右前 RF) =====
#define STEER_2_PWM     (TCPWM_CH25_P09_1)            //控制引脚 ATOM0_CH1_P21_3
#define STEER_2_FRE     (300             )            //PWM频率(Hz)
#define STEER_2_DIR     (-1              )            //旋转方向(-1:反向)
#define STEER_2_CENTER  (4500            )            //中位PWM值

// ===== 舵机3(左后 LB) =====
#define STEER_3_PWM     (TCPWM_CH24_P09_0)            //控制引脚 ATOM0_CH2_P21_4
#define STEER_3_FRE     (300             )            //PWM频率(Hz)
#define STEER_3_DIR     (-1              )            //旋转方向(-1:反向)
#define STEER_3_CENTER  (4500            )            //中位PWM值

// ===== 舵机4(右后 RB) =====
#define STEER_4_PWM     (TCPWM_CH31_P10_3)            //控制引脚 ATOM0_CH3_P21_5
#define STEER_4_FRE     (300             )            //PWM频率(Hz)
#define STEER_4_DIR     (1               )            //旋转方向(+1:正向)
#define STEER_4_CENTER  (4500            )            //中位PWM值

#define BRIDGE_MAX 800                                  //抬桥最大高度

//舵机控制结构体
typedef struct
{
  pwm_channel_enum  pwm_pin;                            //PWM控制引脚
  int16             control_frequency;                  //PWM频率(Hz)
  int16             steer_dir;                          //舵机旋转方向(+1/-1)
  int16             center_num;                         //舵机中位PWM值
  int16             now_location;                       //舵机当前输出位置
} steer_control_struct;

// === 全局声明 ===
extern steer_control_struct   steer_LF, steer_RF, steer_LB, steer_RB;    //四路舵机实例
extern int BRIDGE_PEAK, BRIDGE_SIGN_LUP, BRIDGE_SIGN_RUP;                //抬桥控制变量

// === 函数声明 ===
void steer_init(void);                                 //舵机初始化(四路PWM配置)
void steer_duty_set(steer_control_struct *control_data, int16 duty);      //直接占空比设置(绝对模式)
void steer_control(steer_control_struct *control_data, int16 move_num);   //增量控制(相对模式)
void dynamic_steer_control(void);                      //动态舵机控制(平衡联动)


#endif