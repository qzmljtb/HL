#include "buzzer.h"

uint8 BUZZER_flag = 0;                                //蜂鸣器标志位(0=关 1=短鸣 2=中鸣 3=长鸣)

void buzzer_isr(void)                                  //蜂鸣器定时中断服务(在PIT中断中调用)
{
    static uint16 t_buzzer = 0;                        //鸣叫计时器
    if (BUZZER_flag != 0)                              //有鸣叫请求
    {
      gpio_set_level(BUZZER_PIN, 1);                   //拉高蜂鸣器引脚
      t_buzzer++;                                      //计时累加
    }
    if (BUZZER_flag == 1)                              //模式1:短鸣(250ms)
    {
      if (t_buzzer == 250)                             //计时到达250
      {
        gpio_toggle_level(BUZZER_PIN);                 //翻转电平(拉低关闭)
        BUZZER_flag = 0;                               //清除标志
        t_buzzer = 0;                                  //清零计时器
      }
    }
    if (BUZZER_flag == 2)                              //模式2:中鸣(500ms)
    {
      if (t_buzzer == 500)                             //计时到达500
      {
        gpio_toggle_level(BUZZER_PIN);                 //翻转电平(拉低关闭)
        BUZZER_flag = 0;                               //清除标志
        t_buzzer = 0;                                  //清零计时器
      }
    }
    if (BUZZER_flag == 3)                              //模式3:长鸣(1000ms)
    {
      if (t_buzzer == 1000)                            //计时到达1000
      {
        gpio_toggle_level(BUZZER_PIN);                 //翻转电平(拉低关闭)
        BUZZER_flag = 0;                               //清除标志
        t_buzzer = 0;                                  //清零计时器
      }
    }
}