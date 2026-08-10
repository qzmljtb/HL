#include "zf_common_headfile.h"                       //逐飞库通用头文件

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

extern uint8 go_flag;                                  //启动标志(外部声明 按键1短按触发)
extern uint8 stop_flag;                                //停止标志(外部声明 按键1长按触发)
extern uint8 roll_balance_enable;                      //Roll平衡开关(0=关闭平路模式 1=开启单边桥模式)

void Key_scan(void);                                   //按键扫描函数(含消抖+短按+长按)


#endif