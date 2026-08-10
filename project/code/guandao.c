#include "guandao.h"
#include "zf_driver_flash.h"                               //逐飞Work Flash驱动(页读写)

// ===== 三科目数据缓冲区 =====
float  yaw_buffer[BUF_TOTAL] = {0};                          //共用缓冲区(30400点)
uint16 segment_count[3][MAX_SEGMENTS] = {0};                 //[科目][段号]采样点数

// ===== 科目属性查找表 =====
const uint16 subject_offset[3]   = {BUF_KEMUYI, BUF_KEMUER, BUF_KEMUSAN};
const uint16 subject_pps[3]      = {PTS_KEMUYI, PTS_KEMUER, PTS_KEMUSAN};
const uint8  subject_segments[3] = {SEG_KEMUYI, SEG_KEMUER, SEG_KEMUSAN};
const char*  subject_name[3]     = {"kemu1", "kemu2", "kemu3"};

// ===== 系统状态 =====
uint8 sys_mode           = 0;
uint8 guandao_flag       = 0;
uint8 record_subject     = 0;
uint8 playback_subject   = 0;
uint8 record_segment     = 0;
uint8 playback_segment   = 0;
uint8 playback_last_seg  = 255;                              //回放时最后一段有数据的段号(255=未初始化)
uint8 playback_state     = 0;
uint8 record_needs_init   = 0;                             //ISR触发:进入记录(留在当前段)
uint8 record_needs_switch  = 0;                             //ISR触发:切换记录段(跳到下一段)
uint8 playback_needs_init  = 0;                             //ISR触发:回放初始化
int16 seg_rotate_duty = 0;                                   //段间等待时原地旋转的duty值
uint8 rotate_flag = 0;                                       //原地转标志:1=pidout_5不参与输出

// 查找科目中最后一段有数据的段号(没有数据返回0)
static uint8 find_last_data_seg(uint8 subject)
{
    int8 s;
    uint8 segs = subject_segments[subject];
    for (s = segs - 1; s >= 0; s--)
    {
        if (segment_count[subject][s] > 0)
            return (uint8)s;
    }
    return 0;
}

void guanfao(int num)                                       //惯导步进函数(ISR每3cm调用)
{
  static uint16 playback_index = 0;                         //回放段内索引
  static uint16 seg_wait = 0;                                //段间等待计时(ms)
  static uint8  yaw_align = 0;                               //偏航对准标志:1=对准中
  static float  align_target = 0;                            //对准目标偏航角
  uint16 offset;                                            //缓冲区基址偏移
  uint16 pps;                                               //每段点数
  uint16 cnt;                                               //当前段采样点数
  uint8  segs;                                              //段数
  uint8  s;                                                 //循环变量

  switch (num)
  {
  // ===== 清零:清除当前记录科目的所有段 =====
  case 0:
    segs = subject_segments[record_subject];
    for (s = 0; s < segs; s++)
    {
      segment_count[record_subject][s] = 0;
    }
    record_segment = 0;
    playback_segment = 0;
    playback_index = 0;
    break;

  // ===== 记录步进:每3cm写入偏航角 =====
  case 1:
    offset = subject_offset[record_subject];
    pps    = subject_pps[record_subject];
    segs   = subject_segments[record_subject];
    if (record_segment >= segs) return;                      //段号越界
    cnt = segment_count[record_subject][record_segment];
    if (cnt >= pps) return;                                  //段已满
    if ((int)car_distances == cnt * 3)                       //距离到达采样点
    {
      yaw_buffer[offset + record_segment * pps + cnt] = euler_angle.yaw;
      segment_count[record_subject][record_segment] = cnt + 1;
    }
    break;

  // ===== 回放步进:每3cm读取偏航角 =====
  case 2:
    offset = subject_offset[playback_subject];
    pps    = subject_pps[playback_subject];
    segs   = subject_segments[playback_subject];
    playback_last_seg = find_last_data_seg(playback_subject); //实时查找最后有数据段

    //-- 段间等待:停车原地旋转2秒 --
    if (seg_wait > 0)
    {
      run_speed = 0;                                         //停车
      car_distances = 0;                                     //里程清零
      seg_wait--;                                            //ISR每1ms调guanfao(2)一次
      if (seg_wait == 0)                                     //旋转结束,找下一段第一个点
      {
        seg_rotate_duty = 0;                                 //停止旋转
        rotate_flag = 0;                                     //恢复pidout_5
        // 找下一个有数据的段
        uint8 next_seg = playback_segment + 1;
        while (next_seg < segs && segment_count[playback_subject][next_seg] == 0)
          next_seg++;
        if (next_seg < segs)
        {
          float first_yaw = yaw_buffer[offset + next_seg * pps];
          if (first_yaw != 0)
          {
            Angle_Z = first_yaw;                             //设目标偏航角
            align_target = first_yaw;
            yaw_align = 1;                                   //进入对准状态
          }
          else
          {
            playback_segment = next_seg;                     //第一个点为0,直接跳
            playback_index = 0;
          }
        }
      }
      return;
    }

    //-- 偏航对准:原地转到下一段第一个点的角度 --
    if (yaw_align)
    {
      run_speed = 0;                                         //原地不动
      car_distances = 0;
      {
        float err = euler_angle.yaw - align_target;
        if (err < 0) err = -err;                             //取绝对值
        if (err < 3.0f)                                      //误差<5度
        {
          yaw_align = 0;
          playback_segment++;                                //跳到下一段
          playback_index = 0;
        }
      }
      return;
    }

    //-- 跳过空段 --
    while (playback_segment < segs
           && segment_count[playback_subject][playback_segment] == 0)
    {
      playback_segment++;
      playback_index = 0;
      car_distances = 0;
    }
    if (playback_segment >= segs)                            //全部段回放完毕
    {
      run_speed = 0;
      guandao_flag = 3;                                     //降为平衡,关转向
      return;
    }

    cnt = segment_count[playback_subject][playback_segment];
    if (playback_index >= cnt)                               //当前段回放完毕
    {
      run_speed = 0;                                         //停车
      car_distances = 0;
      // 统计后面还有几段有记录数据
      {
        uint8 seg_left = 0;
        for (s = playback_segment + 1; s < segs; s++)
        {
          if (segment_count[playback_subject][s] > 0)
            seg_left++;
        }
        if (seg_left >= 2)                                   //后面还有≥2段:原地转
        {
          seg_wait = 1500;
          seg_rotate_duty = 1200;
          rotate_flag = 1;
        }
        else                                                 //最后两段:不转,直接跳
        {
          playback_segment++;
          playback_index = 0;
        }
      }
      return;
    }

    //-- 正常回放:每3cm一个采样点(末端减速防惯性冲过头) --
    if ((int)car_distances == playback_index * 3)
    {
      float val = yaw_buffer[offset + playback_segment * pps + playback_index];
      if (val != 0)
      {
        Angle_Z = val;
        // 速度策略: 最后一段独立速度, 其余段末端减速
        {
          uint16 remain = cnt - playback_index;              //剩余采样点数
          if (playback_segment == playback_last_seg)          //最后一段:独立速度,不减速
            run_speed = LAST_SEG_SPEED;
          else if (remain <= 30)                              //其余段:末端缓减速
            run_speed = 100 + (uint16)((PLAYBACK_SPEED - 100) * remain / 30);
          else
            run_speed = PLAYBACK_SPEED;
        }
      }
      else
      {
        run_speed = 0;
      }
      playback_index++;
    }
    break;

  // ===== 进入记录:从当前段开始(不跳段) =====
  case 3:
    segs = subject_segments[record_subject];
    for (s = 0; s < segs; s++)                               //从当前段向后找第一个空段
    {
      uint8 seg = (record_segment + s) % segs;
      if (segment_count[record_subject][seg] == 0)
      {
        record_segment = seg;
        break;
      }
    }
    if (s == segs)                                           //所有段满,覆盖当前段
    {
      // record_segment 保持不变
    }
    segment_count[record_subject][record_segment] = 0;
    car_distances = 0;
    break;

  // ===== 回放初始化:从段0开始 =====
  case 4:
    playback_segment = 0;
    playback_index = 0;
    playback_last_seg = find_last_data_seg(playback_subject);
    car_distances = 0;
    break;

  // ===== 切换记录段:强制跳到下一段 =====
  case 5:
    segs = subject_segments[record_subject];
    for (s = 1; s <= segs; s++)                              //从当前段的下一段开始找空段
    {
      uint8 seg = (record_segment + s) % segs;
      if (segment_count[record_subject][seg] == 0)
      {
        record_segment = seg;
        break;
      }
    }
    if (s > segs)                                            //所有段满,强制跳到下一段覆盖
    {
      record_segment = (record_segment + 1) % segs;
    }
    segment_count[record_subject][record_segment] = 0;
    car_distances = 0;
    break;
  }
}

// ===== Flash持久化 (基于zf_driver_flash) =====
// Work Flash: sector 0, 96页, 每页2KB(512个uint32)
//   科目一: 页 0~19(数据,20页) + 页20(段计数)
//   科目二: 页21~60(数据,40页) + 页61(段计数)
//   科目三: 页62~81(数据,20页) + 页82(段计数)
//   合计: 83页 / 166KB (Work Flash共192KB, 余13页)

#define FLASH_SEC              0                            //Work Flash扇区号

#define PG_K1_DATA             0                            //科目一数据起始页
#define PG_K1_CNT             20                            //科目一段计数页
#define PG_K2_DATA            21                            //科目二数据起始页
#define PG_K2_CNT             61                            //科目二段计数页
#define PG_K3_DATA            62                            //科目三数据起始页
#define PG_K3_CNT             82                            //科目三段计数页

static const uint8 flash_data_pg[3] = {PG_K1_DATA, PG_K2_DATA, PG_K3_DATA};
static const uint8 flash_cnt_pg[3]  = {PG_K1_CNT,  PG_K2_CNT,  PG_K3_CNT};


void flash_save_subject(uint8 subject)
{
    uint16 offset   = subject_offset[subject];
    uint16 total    = (uint16)subject_segments[subject] * subject_pps[subject];
    uint8  segs     = subject_segments[subject];
    uint8  pg_start = flash_data_pg[subject];
    uint16 remaining = total;
    uint16 buf_off   = 0;
    uint8  p;

    // 按页保存yaw数据(float→uint32,等宽转换)
    for (p = 0; remaining > 0; p++)
    {
        uint16 chunk = (remaining > FLASH_PAGE_LENGTH) ? FLASH_PAGE_LENGTH : remaining;
        flash_write_page(FLASH_SEC, pg_start + p,
                         (const uint32*)&yaw_buffer[offset + buf_off], chunk);
        buf_off   += chunk;
        remaining -= chunk;
    }

    // 保存segment_count(uint16→uint32)
    {
        uint32 cnt_buf[MAX_SEGMENTS];
        uint8 i;
        for (i = 0; i < segs; i++)
        {
            cnt_buf[i] = segment_count[subject][i];
        }
        flash_write_page(FLASH_SEC, flash_cnt_pg[subject], cnt_buf, segs);
    }
}

static void flash_load_subject(uint8 subject)
{
    uint16 offset   = subject_offset[subject];
    uint16 total    = (uint16)subject_segments[subject] * subject_pps[subject];
    uint8  segs     = subject_segments[subject];
    uint8  pg_start = flash_data_pg[subject];
    uint16 remaining = total;
    uint16 buf_off   = 0;
    uint8  p;

    // 按页加载yaw数据
    for (p = 0; remaining > 0; p++)
    {
        uint16 chunk = (remaining > FLASH_PAGE_LENGTH) ? FLASH_PAGE_LENGTH : remaining;
        flash_read_page(FLASH_SEC, pg_start + p,
                        (uint32*)&yaw_buffer[offset + buf_off], chunk);
        buf_off   += chunk;
        remaining -= chunk;
    }

    // 加载segment_count(uint32→uint16)
    {
        uint32 cnt_buf[MAX_SEGMENTS];
        uint8 i;
        flash_read_page(FLASH_SEC, flash_cnt_pg[subject], cnt_buf, segs);
        for (i = 0; i < segs; i++)
        {
            segment_count[subject][i] = (uint16)cnt_buf[i];
        }
    }
}

void flash_load_all(void)
{
    uint8 s;
    for (s = 0; s < MAX_SUBJECTS; s++)
    {
        flash_load_subject(s);
    }
}