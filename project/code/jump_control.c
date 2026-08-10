#include "jump_control.h"

// ===== 全局变量 =====
int             jump_Flag   = 0;
jump_state_t    jump_state  = JUMP_IDLE;
float           jump_dist1  = 0;    // 跳①→跳② 间隔里程(cm)
float           jump_dist2  = 0;    // 跳②→跳③ 间隔里程(cm)

// ===== 内部静态变量 =====
static uint16   jtimer      = 0;    // 通用1ms tick计数器(各状态复用)
static uint8    jstep       = 0;    // 当前跳跃步号(0~4)
static uint8    jcount      = 0;    // 已完成跳跃次数(连跳计数)
static float    saved_speed = 0;    // 跳前目标速度(跳完恢复)

// ===== 内部辅助函数 =====

// 检查车身角度是否在安全范围内
static inline uint8 angle_ok(void)
{
    return (euler_angle.pitch >= -JUMP_PITCH_MAX && euler_angle.pitch <= JUMP_PITCH_MAX
         && euler_angle.roll  >= -JUMP_ROLL_MAX  && euler_angle.roll  <= JUMP_ROLL_MAX);
}

// 停车并进入跳跃执行状态
static void enter_execute(void)
{
    saved_speed = run_speed;
    run_speed   = 0;
    jump_state  = JUMP_EXECUTE;
    jtimer      = 0;
    jstep       = 1;
    jump_step_a(1);
}

// 取消跳跃: 恢复速度 → 回到空闲
static void abort_jump(void)
{
    run_speed  = saved_speed;
    jump_state = JUMP_IDLE;
    jump_Flag  = 0;
}

// 跳跃里程累加(仅WAIT_RETRY期间, 按jcount分别累加)
static void dist_tick(void)
{
    if (jump_state != JUMP_WAIT_RETRY) return;

    float spd  = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
    float step = ((int)spd / 60.0f * 6.5f * PI * 0.001f);

    if      (jcount == 1) jump_dist1 += step;
    else if (jcount == 2) jump_dist2 += step;
}

// 距离达标判断
static uint8 dist_ok(void)
{
    if (jcount == 1 && jump_dist1 >= JUMP_RETRY1_DISTANCE) return 1;
    if (jcount == 2 && jump_dist2 >= JUMP_RETRY2_DISTANCE) return 1;
    return 0;
}

// 清零即将使用的里程变量
static void dist_reset(void)
{
    if      (jcount == 1) jump_dist1 = 0;
    else if (jcount == 2) jump_dist2 = 0;
}

// 跳跃步进序列(时间驱动)
static void exec_tick(void)
{
    jtimer++;

    if (jtimer >= step_0_stop && jstep == 0) { jstep = 1; jump_step_a(1); }
    if (jtimer >= step_1_stop && jstep == 1) { jstep = 2; jump_step_a(2); }
    if (jtimer >= step_2_stop && jstep == 2) { jstep = 3; jump_step_a(3); }
    if (jtimer >= step_3_stop && jstep == 3) {              jump_step_a(4); }

    // 跳跃序列结束
    if (jtimer >= step_4_stop)
    {
        jcount++;
        if (jcount < JUMP_MAX_COUNT)
        {
            run_speed  = 0;
            jump_state = JUMP_STABILIZE;
            jtimer     = 0;
        }
        else
        {
            run_speed  = saved_speed;
            jstep      = 0;
            jtimer     = 0;
            jump_state = JUMP_IDLE;
            jump_Flag  = 0;
        }
    }
}

// ===== 公开API =====

// 按键触发跳跃(camera.c中调用)
void jump_trigger(void)
{
    if (jump_state == JUMP_IDLE && angle_ok())
    {
        jump_state = JUMP_WAIT_2S;
        jtimer     = 0;
        jcount     = 0;
        jump_dist1 = 0;
        jump_dist2 = 0;
        jump_Flag  = 1;
    }
}

// 跳跃状态机(timer.c中每1ms调用)
void jump_control(void)
{
    dist_tick();  // 里程累加(WAIT_RETRY期间)

    switch (jump_state)
    {
    // ---- 空闲 ----
    case JUMP_IDLE:
        break;

    // ---- 起跳前等待 ----
    case JUMP_WAIT_2S:
        jtimer++;
        if (jtimer >= JUMP_WAIT_TICKS)
        {
            if (!angle_ok()) { jump_state = JUMP_IDLE; jump_Flag = 0; }
            else              enter_execute();
        }
        break;

    // ---- 跳后稳定 ----
    case JUMP_STABILIZE:
        jtimer++;
        if (jtimer >= JUMP_STABILIZE_TICKS)
        {
            run_speed  = saved_speed;
            dist_reset();
            jump_state = JUMP_WAIT_RETRY;
            jtimer     = 0;
        }
        break;

    // ---- 连跳间隔(距离触发) ----
    case JUMP_WAIT_RETRY:
        jtimer++;
        if (dist_ok())
        {
            if (!angle_ok()) abort_jump();
            else             enter_execute();
        }
        break;

    // ---- 跳跃执行(时间驱动步进) ----
    case JUMP_EXECUTE:
        exec_tick();
        break;

    default:
        break;
    }
}

// 跳跃步动作(舵机序列)
void jump_step_a(int step_num)
{
    switch (step_num)
    {
    case 0:  // 深蹲蓄力(已取消,保留占位)
        steer_duty_set(&steer_LF, steer_LF.center_num - 2000);
        steer_duty_set(&steer_RF, steer_RF.center_num + 2000);
        steer_duty_set(&steer_LB, steer_LB.center_num + 2000);
        steer_duty_set(&steer_RB, steer_RB.center_num - 2000);
        break;

    case 1:  // 爆发蹬地起跳
        steer_duty_set(&steer_LF, steer_LF.center_num + 2000);
        steer_duty_set(&steer_RF, steer_RF.center_num - 2000);
        steer_duty_set(&steer_LB, steer_LB.center_num - 2000);
        steer_duty_set(&steer_RB, steer_RB.center_num + 2000);
        break;

    case 2:  // 空中收腿
        steer_duty_set(&steer_LF, steer_LF.center_num - 1500);
        steer_duty_set(&steer_RF, steer_RF.center_num + 1500);
        steer_duty_set(&steer_LB, steer_LB.center_num + 1500);
        steer_duty_set(&steer_RB, steer_RB.center_num - 1500);
        break;

    case 3:  // 落地预备伸腿触地
        steer_duty_set(&steer_LF, steer_LF.center_num + 0);
        steer_duty_set(&steer_RF, steer_RF.center_num - 0);
        steer_duty_set(&steer_LB, steer_LB.center_num - 0);
        steer_duty_set(&steer_RB, steer_RB.center_num + 0);
        break;

    case 4:  // 落地缓冲吸收
        steer_duty_set(&steer_LF, steer_LF.center_num - 500);
        steer_duty_set(&steer_RF, steer_RF.center_num + 500);
        steer_duty_set(&steer_LB, steer_LB.center_num + 500);
        steer_duty_set(&steer_RB, steer_RB.center_num - 500);
        break;

    default:
        break;
    }
}