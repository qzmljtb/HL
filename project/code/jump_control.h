#include "zf_common_headfile.h"

#ifndef JUMP_CONTROL_H_
#define JUMP_CONTROL_H_

extern int jump_Flag;

// 跳跃步时间阈值(每个tick=1ms,慢舵机版,无深蹲直接蹬地)
// Step1: 蹬地起跳  Step2: 空中收腿  Step3: 落地预备  Step4: 深蹲缓冲
#define step_0_start    0
#define step_0_stop     0
#define step_1_stop     step_0_stop + 110
#define step_2_stop     step_1_stop + 130
#define step_3_stop     step_2_stop + 30
#define step_4_stop     step_3_stop + 250

#define JUMP_WAIT_TICKS       1000    // 首次延时 2000ms = 2s
#define JUMP_STABILIZE_TICKS  0    // 跳后停车稳定
#define JUMP_RETRY1_TICKS     600    // 跳①→跳② 间隔 600ms(第一跳用时间,保留)
#define JUMP_RETRY2_TICKS     600    // 跳②→跳③ 间隔 600ms(第一跳用时间,保留)
#define JUMP_RETRY1_DISTANCE   20    // 跳①→跳② 行驶距离(cm) 第二跳用车距触发
#define JUMP_RETRY2_DISTANCE   22   // 跳②→跳③ 行驶距离(cm) 第三跳用车距触发
#define JUMP_MAX_COUNT           1    // 连跳3次
#define JUMP_PITCH_MAX          30    // 跳跃俯仰角保护阈值(超过禁止起跳)
#define JUMP_ROLL_MAX           35    // 跳跃横滚角保护阈值(超过禁止起跳)

// 跳跃状态机枚举
typedef enum {
    JUMP_IDLE = 0,               // 空闲:等待按键触发
    JUMP_WAIT_2S,                // 首次2秒倒计时
    JUMP_EXECUTE,                // 正在执行跳跃序列
    JUMP_STABILIZE,              // 跳后停车稳定1s
    JUMP_WAIT_RETRY,             // 连跳间隔等待
} jump_state_t;

extern jump_state_t jump_state;  // 跳跃状态(供timer.c判断是否正在执行跳跃)
extern float jump_dist1;         // 跳①→跳② 间隔里程(cm)
extern float jump_dist2;         // 跳②→跳③ 间隔里程(cm)

void jump_step_a(int step_num);
void jump_trigger(void);         // 按键触发跳跃(外部调用)
void jump_control(void);         // 跳跃状态机(在2ms定时中断中调用)

#endif