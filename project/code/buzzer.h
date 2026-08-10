#include "zf_common_headfile.h"

#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_

#define BUZZER_PIN              (P19_4)                //蜂鸣器引脚(P19_4)

extern uint8 BUZZER_flag;                              //蜂鸣器标志位(外部声明 0=关 1=短 2=中 3=长)

void buzzer_isr(void);                                 //蜂鸣器定时中断服务函数


#endif