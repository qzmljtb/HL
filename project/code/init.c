#include "init.h"
#include "zf_driver_flash.h"                           //Work Flash驱动

float car_distances = 0;                            //小车行驶距离

void INIT(void)                                     //系统总初始化
{
  gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);     //蜂鸣器GPIO初始化

  imu660rb_init();                                  //IMU660RB陀螺仪初始化

  //  imu963ra_init();                                 //IMU963RA九轴陀螺仪(备用)
  //  imu660ra_init();                                 //IMU660RA陀螺仪(备用)
  //  imu660rc_init(IMU660RC_QUARTERNION_DISABLE);      //IMU660RC陀螺仪(备用)

  EKF_Init();                                       //扩展卡尔曼滤波初始化

  PID_param_init();                                 //PID参数初始化(平衡环+速度环+转向环)

  ips200_set_dir(IPS200_CROSSWISE_180);             //IPS200屏幕方向(横屏翻转180°)
  
  ips200_init(IPS200_TYPE_SPI);                     //IPS200屏幕SPI初始化

  mt9v03x_init();                                 //摄像头初始化
  
  key_init(2);                                      //按键初始化(2个按键)
  
  steer_init();                                     //四路舵机PWM初始化
  
  small_driver_uart_init();                         //小驱动板串口通信初始化

  flash_init();                                      //Work Flash驱动初始化

  pit_init(PIT_CH0, 1000);                          //PIT通道0初始化(1ms中断)
  pit_init(PIT_CH1, 1000);                          //PIT通道1初始化(1ms中断)
  pit_init(PIT_CH2, 1000);                          //PIT通道2初始化(1ms中断)
}