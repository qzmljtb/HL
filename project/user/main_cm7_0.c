#include "zf_common_headfile.h"
#include "guandao.h"                                  //惯导模块(flash_load_all等)


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留
    debug_init();                       // 调试串口信息初始化
    INIT();
    flash_load_all();                                      //上电从Flash恢复全部三科目数据

    while(true)
    {
        if (Flag.image_process_ready)
        {
            ImageProcess();
            xunzhongxian();
            switch(task){
                case 1:
                    ips200_show_gray_image(0, 0, erzhi_image[0], LCDW,LCDH, LCDW, LCDH, 0); 
                    break;
                case 2:
                    ips200_show_gray_image(0, 0, erzhi_image[0], LCDW,LCDH, LCDW, LCDH, 0); 
                    break;
                case 3:
                    ips200_show_gray_image(0, 0, temp_image[0], LCDW,LCDH, LCDW, LCDH, 0);
                    break;
                default:
                    break;
            }
            Flag.image_process_ready = 0;
        }
      ips200_show_string(8*0,  16*0, "R:");
      ips200_show_float (8*2,  16*0, euler_angle.roll, 4, 2);
      ips200_show_string(8*15, 16*0, "P:");
      ips200_show_float (8*17, 16*0, euler_angle.pitch, 4, 2);
      ips200_show_string(8*30, 16*0, "Y:");
      ips200_show_float (8*32, 16*0, euler_angle.yaw, 4, 2);
      ips200_show_string(8*0,  16*1, "GO:");
      ips200_show_int   (8*3,  16*1, go_flag, 2);
      ips200_show_int   (8*0,  16*2, (int)car_distances, 5);
      ips200_show_string(8*7,  16*2, "F:");
      ips200_show_int   (8*9,  16*2, guandao_flag, 1);
      ips200_show_int   (8*9,  16*3, step_result.jump_now, 1);

      // ===== 第3行:陀螺仪零飘状态 =====
      ips200_show_string(8*0,  16*3, "GYRO:");
      if (gyro_calib_ok)
          ips200_show_string(8*5,  16*3, "OK ");
      else
          ips200_show_string(8*5,  16*3, "---");

      // ===== 第5行:模式+状态 =====
      {
          const char* mode_str;                              //模式名称
          const char* state_str;                             //状态名称

          if (sys_mode == 0)
          {
              mode_str  = "normal";
              state_str = "idle  ";
          }
          else if (sys_mode == 1)                            //记录模式
          {
              mode_str  = "record";
              state_str = (guandao_flag == 1) ? "run   " : "init  ";
          }
          else                                               //回放模式
          {
              mode_str  = "play  ";
              if (playback_state == 0)
                  state_str = "init  ";
              else if (playback_state == 1)
                  state_str = "balance";
              else
                  state_str = "run   ";
          }
          ips200_show_string(8*0,  16*5, "M:");             //模式
          ips200_show_string(8*2,  16*5, mode_str);
          ips200_show_string(8*12, 16*5, "S:");             //状态
          ips200_show_string(8*14, 16*5, state_str);
      }

      // ===== 第6行:记录/回放所选科目 =====
      {
          ips200_show_string(8*0,  16*6, "R:");             //记录科目
          ips200_show_string(8*2,  16*6, subject_name[record_subject]);
          ips200_show_string(8*12, 16*6, "P:");             //回放科目
          ips200_show_string(8*14, 16*6, subject_name[playback_subject]);
      }

      // ===== 第7行:段号/附加信息 =====
      if (sys_mode == 1)                                     //记录模式:显示段号
      {
          ips200_show_string(8*0,  16*7, "SEG:");
          ips200_show_int   (8*4,  16*7, record_segment, 2);
      }
      else if (sys_mode == 2)                                //回放模式:显示段号
      {
          ips200_show_string(8*0,  16*7, "SEG:");
          ips200_show_int   (8*4,  16*7, playback_segment, 2);
      }
    }
}

