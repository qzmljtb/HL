#include "zf_common_headfile.h"

#ifndef CODE_GUANDAO_H_
#define CODE_GUANDAO_H_

// ===== 三科目缓冲区布局 =====
// 科目一: 偏移0      1段×10000点 = 10000点
// 科目二: 偏移10000  10段×2000点 = 20000点
// 科目三: 偏移30000  1段×10000点 = 10000点
// 总计: 40000点 ≈ 160KB

#define BUF_KEMUYI          0                               //科目一缓冲区起始偏移
#define BUF_KEMUER          10000                           //科目二缓冲区起始偏移
#define BUF_KEMUSAN         30000                           //科目三缓冲区起始偏移

#define PTS_KEMUYI          10000                           //科目一每段点数
#define PTS_KEMUER           2000                           //科目二每段点数
#define PTS_KEMUSAN          10000                           //科目三每段点数

#define SEG_KEMUYI              1                           //科目一段数
#define SEG_KEMUER             10                           //科目二段数
#define SEG_KEMUSAN             1                           //科目三段数

#define MAX_SEGMENTS           10                           //最大段数(科目二)
#define PLAYBACK_SPEED          500                           //科目二回放默认速度(可调)
#define LAST_SEG_SPEED          1000                           //科目二最后一段专用速度(可调)
#define MAX_SUBJECTS            3                           //科目总数
#define BUF_TOTAL           40000                           //缓冲区总点数

extern float  yaw_buffer[BUF_TOTAL];                         //三科目共用数据缓冲区
extern uint16 segment_count[3][MAX_SEGMENTS];                //[科目][段号] 每段实际采样点数

// ===== 科目属性查找表 =====
extern const uint16 subject_offset[3];                       //每个科目的缓冲区偏移
extern const uint16 subject_pps[3];                          //每个科目的每段点数
extern const uint8  subject_segments[3];                     //每个科目的段数
extern const char*  subject_name[3];                         //每个科目的名称

// ===== 系统状态 =====
extern uint8 sys_mode;                                       //系统模式: 0=普通 1=记录 2=回放
extern uint8 guandao_flag;                                   //ISR控制: 0=空闲 1=记录 2=清零 3=平衡 4=回放
extern uint8 record_subject;                                 //当前选择的记录科目(0~2)
extern uint8 playback_subject;                               //当前选择的回放科目(0~2)
extern uint8 record_segment;                                 //当前记录段号
extern uint8 playback_segment;                               //当前回放段号
extern uint8 playback_last_seg;                              //最后一段有数据的段号(回放初始化时计算)
extern uint8 playback_state;                                 //回放子状态: 0=待机 1=平衡 2=回放中
extern uint8 record_needs_init;                              //ISR触发:调用guanfao(3)进入记录(留在当前段)
extern uint8 record_needs_switch;                            //ISR触发:调用guanfao(5)切换记录段(跳到下一段)
extern uint8 playback_needs_init;                            //ISR触发:调用guanfao(4)初始化回放
extern int16 seg_rotate_duty;                                 //段间等待时原地旋转duty(timer.c读取)
extern uint8 rotate_flag;                                     //原地转标志:1=pidout_5不参与电机输出

// ===== API =====
void guanfao(int num);                                       //惯导步进函数(ISR调用)
void flash_load_all(void);                                   //上电时从Flash加载全部三科目
void flash_save_subject(uint8 subject);                      //保存单个科目到Flash


#endif