#include "key.h"                                      //按键模块头文件
#include "guandao.h"                                  //惯导模块(状态变量+API)

uint8 go_flag = 0;                                     //启动标志(按键触发)
uint8 stop_flag = 0;                                   //停止标志(按键触发)
uint8 roll_balance_enable = 0;                         //Roll平衡开关(0=平路模式 1=单边桥模式)

void Key_scan(void)                                    //按键扫描函数(在定时中断或主循环中调用)
{
    key_scanner();                                     //逐飞库按键状态扫描(已含硬件消抖)

    // ==================== 按键1 ====================
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        BUZZER_flag = 1;                               //蜂鸣器反馈(第一步)
        if (sys_mode == 0)                             //普通:切换记录科目
        {
            record_subject = (record_subject + 1) % MAX_SUBJECTS;
        }
        else if (sys_mode == 1)                        //记录:切到下一段
        {
            uint8 segs = subject_segments[record_subject];
            uint8 s;
            for (s = 1; s <= segs; s++)                //从下一段找空段
            {
                uint8 seg = (record_segment + s) % segs;
                if (segment_count[record_subject][seg] == 0)
                {
                    record_segment = seg;
                    break;
                }
            }
            if (s > segs)                              //全满则覆盖下一段
            {
                record_segment = (record_segment + 1) % segs;
            }
            segment_count[record_subject][record_segment] = 0;
            car_distances = 0;
            guandao_flag = 1;
        }
        else if (sys_mode == 2)                        //回放:待机→平衡→回放
        {
            if (playback_state == 0)                   //第1次:平衡
            {
                playback_segment = 0;
                car_distances = 0;
                guandao_flag = 3;
                playback_state = 1;
            }
            else if (playback_state == 1)              //第2次:回放
            {
                guandao_flag = 4;
                playback_state = 2;
            }
        }
    }
    if (key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        BUZZER_flag = 3;                               //蜂鸣器反馈(第一步)
        if (sys_mode == 0)                             //普通→进记录
        {
            uint8 segs = subject_segments[record_subject];
            uint8 s;
            sys_mode = 1;
            for (s = 0; s < segs; s++)                 //清空所有段计数(避免残留)
                segment_count[record_subject][s] = 0;
            for (s = 0; s < segs; s++)                 //从当前段找第一个空段
            {
                uint8 seg = (record_segment + s) % segs;
                if (segment_count[record_subject][seg] == 0)
                {
                    record_segment = seg;
                    break;
                }
            }
            segment_count[record_subject][record_segment] = 0;
            car_distances = 0;
            guandao_flag = 1;
        }
        else if (sys_mode == 2)                        //回放→退出
        {
            guandao_flag = 0;
            run_speed = 0;
            playback_state = 0;
            sys_mode = 0;
        }
    }

    // ==================== 按键2 ====================
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
    BUZZER_flag = 1;
        if (sys_mode == 0)                             //普通:切换回放科目
        {
            playback_subject = (playback_subject + 1) % MAX_SUBJECTS;
        }
        else if (sys_mode == 1)                        //记录:清空当前段
        {
            segment_count[record_subject][record_segment] = 0;
            car_distances = 0;
        }
    }
    if (key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
    BUZZER_flag = 3;
        if (sys_mode == 0)                             //普通→进回放
        {
            sys_mode = 2;
            guandao_flag = 0;
            playback_state = 0;
            playback_segment = 0;
            car_distances = 0;
        }
        else if (sys_mode == 1)                        //记录→退出保存
        {
            guandao_flag = 0;
            flash_save_subject(record_subject);
            sys_mode = 0;
        }
    }

    // ==================== 按键3 ====================
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
          BUZZER_flag = 1;
          jump_trigger();                            //触发跳跃状态机
        }
    if (key_get_state(KEY_3) == KEY_LONG_PRESS)
        { BUZZER_flag = 2; }

    // ==================== 按键4 ====================
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
          BUZZER_flag = 1;
          go_flag++;                                 //启动标志+1
          if(go_flag == 3)
          {
            run_speed = 170;
            zhongxian_flag = 1;
          }
        }
    if (key_get_state(KEY_4) == KEY_LONG_PRESS)
        { BUZZER_flag = 2; }
}