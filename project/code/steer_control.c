#include "steer_control.h"                            //舵机控制头文件
#include "zf_common_headfile.h"                       //逐飞库通用头文件

//四路舵机实例
steer_control_struct   steer_LF, steer_RF, steer_LB, steer_RB;
//LF=左前腿 LB=左后腿 RF=右前腿 RB=右后腿

//抬桥相关变量
int BRIDGE_PEAK = 0;                                  //桥峰高度
int BRIDGE_SIGN_LUP = 0;                              //左抬桥标志
int BRIDGE_SIGN_RUP = 0;                              //右抬桥标志

//舵机初始化(四路PWM配置)
void steer_init(void)
{
  // ===== 基础参数配置 =====
  steer_LF.pwm_pin           = STEER_1_PWM;           //控制引脚
  steer_LF.control_frequency = STEER_1_FRE;           //频率(300Hz)
  steer_LF.steer_dir         = STEER_1_DIR;           //旋转方向(+1=正向)
  steer_LF.center_num        = STEER_1_CENTER;        //中心值PWM(4525)

  steer_RF.pwm_pin           = STEER_2_PWM;           //控制引脚
  steer_RF.control_frequency = STEER_2_FRE;           //频率(300Hz)
  steer_RF.steer_dir         = STEER_2_DIR;           //旋转方向(-1=反向)
  steer_RF.center_num        = STEER_2_CENTER;        //中心值PWM(4500)

  steer_LB.pwm_pin           = STEER_3_PWM;           //控制引脚
  steer_LB.control_frequency = STEER_3_FRE;           //频率(300Hz)
  steer_LB.steer_dir         = STEER_3_DIR;           //旋转方向(-1=反向)
  steer_LB.center_num        = STEER_3_CENTER;        //中心值PWM(4500)

  steer_RB.pwm_pin           = STEER_4_PWM;           //控制引脚
  steer_RB.control_frequency = STEER_4_FRE;           //频率(300Hz)
  steer_RB.steer_dir         = STEER_4_DIR;           //旋转方向(+1=正向)
  steer_RB.center_num        = STEER_4_CENTER;        //中心值PWM(4675)

  // ===== 当前位置初始化为中位 =====
  steer_LF.now_location = steer_LF.center_num;
  steer_RF.now_location = steer_RF.center_num;
  steer_LB.now_location = steer_LB.center_num;
  steer_RB.now_location = steer_RB.center_num;

  // ===== 四路PWM初始化 =====
  pwm_init(steer_LF.pwm_pin, steer_LF.control_frequency, steer_LF.now_location);
  pwm_init(steer_RF.pwm_pin, steer_RF.control_frequency, steer_RF.now_location);
  pwm_init(steer_LB.pwm_pin, steer_LB.control_frequency, steer_LB.now_location);
  pwm_init(steer_RB.pwm_pin, steer_RB.control_frequency, steer_RB.now_location);
}

//舵机直接占空比设置(绝对位置模式)
void steer_duty_set(steer_control_struct *control_data, int16 duty)
{
  control_data->now_location = func_limit_ab(duty, -10000, 10000);  //限幅±10000
  pwm_set_duty(control_data->pwm_pin, duty);                         //输出PWM
}

//舵机增量控制(相对位移模式 自动处理方向)
void steer_control(steer_control_struct *control_data, int16 move_num)
{
  //累加增量(根据方向自动取反)
  control_data->now_location = control_data->now_location + (control_data->steer_dir == 1 ? move_num : -move_num);
  control_data->now_location = func_limit_ab(control_data->now_location, -10000, 10000); //限幅±10000
  pwm_set_duty(control_data->pwm_pin, control_data->now_location);                        //输出PWM
}

//动态舵机控制(根据平衡PID输出实时调整四条腿)
void dynamic_steer_control(void)
{
 int16 steer_output_duty = 0;                                //舵机输出占空比(前后俯仰分量)
 int16 steer_location_offset[4] = {0};                        //四路当前位置偏移量(相对中位)
 int16 steer_target_offset[4] = {0};                          //四路目标位置偏移量

 static float steer_output_duty_filter = 0;                   //俯仰分量一阶低通滤波输出
 static float roll_output_duty_filter = 0;                    //横滚分量一阶低通滤波输出

 //从俯仰PID输出(pitch_speed_wheel)推导舵机前后控制量
 //pidout_1 / 7 → 缩放到[-350,350] → ×6 → 取反
 steer_output_duty = -func_limit_ab(pidout_1 / 7, -350, 350) * 6;
 //一阶低通滤波: y(n) = 0.95*y(n-1) + 0.05*x(n)
 steer_output_duty_filter = (steer_output_duty_filter * 19 + (float)steer_output_duty) / 20.0f;

 //从横滚PID输出(roll_angle_leg)推导舵机左右控制量
 //仅在Roll平衡开关打开时生效(平路关闭 单边桥开启)
 if (roll_balance_enable)
 {
     int16 roll_output_duty = func_limit_ab(pidout_roll_leg / 5, -1500, 1500);
     //一阶低通滤波(平滑腿部动作) alpha=0.1
     roll_output_duty_filter = (roll_output_duty_filter * 9 + (float)roll_output_duty) / 10.0f;
 }
 else
 {
     roll_output_duty_filter = 0;  //平路模式:关闭Roll差动,腿只响应Pitch
 }

 // ===== 计算当前位置相对中位的偏移量 =====
 steer_location_offset[0] = (steer_LF.now_location - steer_LF.center_num) * steer_LF.steer_dir;
 steer_location_offset[1] = (steer_RF.now_location - steer_RF.center_num) * steer_RF.steer_dir;
 steer_location_offset[2] = (steer_LB.now_location - steer_LB.center_num) * steer_LB.steer_dir;
 steer_location_offset[3] = (steer_RB.now_location - steer_RB.center_num) * steer_RB.steer_dir;

 // ===== 计算目标偏移(Pitch前后差动 + Roll左右差动) =====
 // Pitch: 前腿同向、后腿反向(前后平衡)
 // Roll:  左腿同向、右腿反向(左右平衡)
 // 叠加: LF = -pitch - roll   RF = -pitch + roll
 //       LB = +pitch - roll   RB = +pitch + roll
 steer_target_offset[0] =   -steer_output_duty_filter - roll_output_duty_filter;  //左前
 steer_target_offset[1] =   -steer_output_duty_filter + roll_output_duty_filter;  //右前
 steer_target_offset[2] =    steer_output_duty_filter - roll_output_duty_filter;  //左后
 steer_target_offset[3] =    steer_output_duty_filter + roll_output_duty_filter;  //右后

 // ===== 增量逼近目标(每次最多±10步 避免突变) =====
     steer_control(&steer_LF, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -10, 10));
     steer_control(&steer_RF, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -10, 10));
     steer_control(&steer_LB, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -10, 10));
     steer_control(&steer_RB, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -10, 10));
}