#include "zf_common_headfile.h"

#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#define WHEEL__diameter  6.5                         //车轮直径(cm)

extern float car_distances;                          //小车行驶距离(外部声明)

//程序声明
void INIT(void);                                     //系统总初始化

#endif