#include "zf_common_headfile.h"                       //逐飞库通用头文件

#ifndef CODE_TIMER_H_
#define CODE_TIMER_H_

//定时器时基标志位结构体(各周期在PIT中断中置位 主函数中轮询)
typedef struct
{
    unsigned char T_2ms;                               //2ms标志位(500Hz)
    unsigned char T_8ms;                               //8ms标志位(125Hz)
    unsigned char T_10ms;                              //10ms标志位(100Hz)
    unsigned char T_20ms;                              //20ms标志位(50Hz)
    unsigned char T_50ms; 
    unsigned char T_100ms;                             //100ms标志位(10Hz)
    unsigned char T_1s;                                //1s标志位(1Hz)
    unsigned char image_process_ready;                  //图像处理就绪标志(ISR置位→主循环执行)
} Timer_0;

extern Timer_0 Flag;                                   //时基标志位(外部声明)

void Fuse_result(void);                                //控制融合主函数(核心调度 在PIT中断中调用)


#endif