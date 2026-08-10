#include "timer.h"                                    //定时器/控制调度头文件

Timer_0 Flag;                                         //时基标志位实例

//控制融合主函数(在PIT中断中调用 核心调度)
void Fuse_result(void)
{
    // ===== 2ms控制周期(500Hz 最高频) =====
    if (Flag.T_2ms)
    {
        Key_scan();                                    //按键扫描
        getangle();                                    //获取IMU姿态角

        uint8 motor_active = (go_flag >= 1 || guandao_flag >= 3);  //电机使能条件
        uint8 steer_active = (go_flag >= 2 || guandao_flag >= 4);  //转向使能条件

        if (motor_active
            && euler_angle.pitch >= -30 && euler_angle.pitch <= 30
            && euler_angle.roll  >= -35 && euler_angle.roll  <= 35)
        {
            // === 俯仰串级PID: 速度环(外) → 角速度环(内) ===
            pidout_1 = PITCH_SPEED_LOOP(0);
            pidout_3 = PITCH_ANGLESPEED_LOOP(pidout_2);
            // === 偏航串级PID: 角速度环(内) ===
            pidout_5 = YAW_ANGLESPEED_LOOP(pidout_4);

            //电机差动输出(rotate_flag=1时关pidout_5,加seg_rotate_duty原地转)
            if (steer_active)
            {
                if (rotate_flag)
                    small_driver_set_duty(-(pidout_3 - seg_rotate_duty), -(pidout_3 + seg_rotate_duty));
                else
                    small_driver_set_duty(-(pidout_3 - pidout_5), -(pidout_3 + pidout_5));
            }
            else
                small_driver_set_duty(-pidout_3, -pidout_3);
                
            // 跳跃执行期间跳过舵机联动,让跳跃状态机独占舵机
            if (jump_state != JUMP_EXECUTE)
                dynamic_steer_control();                   //四条腿舵机联动
        }
        else if (motor_active)                           //角度超限保护:倾斜过大时停止电机
        {
            small_driver_set_duty(0, 0);                 //电机停止输出
            run_speed = 0;                               //目标速度清零
        }

        Flag.T_2ms = 0;
    }
    // ===== 8ms控制周期(125Hz) =====
    else if (Flag.T_8ms)
    {
        if (go_flag >= 1 || guandao_flag >= 3)
        {
            // === 角度环PID(外层 低频计算) ===
            pidout_2 = PITCH_ANGLE_LOOP(0);            //俯仰角度环→角速度期望
            pidout_4 = YAW_ANGLE_LOOP(vision_yaw_target);              //偏航角度环→偏航角速度期望
            pidout_roll_leg = ROLL_ANGLE_LEG_LOOP(0);  //横滚角度环→腿部左右差动量
        }
        Flag.T_8ms = 0;
    }
    // ===== 低频时基(预留扩展) =====
    else if (Flag.T_10ms)
    {
        Flag.T_10ms  = 0;
    }
    else if (Flag.T_20ms)
    {
        Flag.T_20ms  = 0;
    }
    else if (Flag.T_50ms)
    {
        if(mt9v03x_finish_flag)
        {
            memcpy(base_image[0],mt9v03x_image[0],MT9V03X_IMAGE_SIZE);//复制原图像
            Flag.image_process_ready = 1;                              //只置标志，主循环中处理
            mt9v03x_finish_flag = 0;
        }
        Flag.T_50ms  = 0;
    }
    else if (Flag.T_100ms)
    {
        Flag.T_100ms = 0;
    }
    else if (Flag.T_1s)
    {
        Flag.T_1s    = 0;
    }
    jump_control();
}