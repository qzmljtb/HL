#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "zf_common_headfile.h"

#define LCDW            94
#define LCDH            60
//============================================================================
// 外部变量声明
//============================================================================

extern seekfree_assistant_transfer_callback_function   seekfree_assistant_transfer_callback;    // 数据发送回调函数指针
extern seekfree_assistant_receive_callback_function    seekfree_assistant_receive_callback;     // 数据接收回调函数指针
extern float vision_yaw_target;                         //视觉偏航角目标(°): P+I输出, YAW_ANGLE_LOOP的设定值
extern float vision_yaw_i;                              //视觉偏航积分项(°): 累积中线偏差消除稳态误差
//============================================================================
// 配置区（按实际硬件参数修改）
//============================================================================

// 视觉偏航 P+I 控制参数（像素偏差 → 偏航角度）
#define VISION_YAW_KP         0.80f       // P增益: (°/像素) 偏差立即映射为偏航角
#define VISION_YAW_KI         0.08f       // I增益: (°/像素/50ms) 累积消除稳态误差
#define VISION_YAW_I_MAX      25.0f       // 积分限幅(°): 正常运行时防windup
#define VISION_YAW_MAX        45.0f       // 偏航目标总限幅(°): 正常运行

// 自适应饱和解除: 目标持续撞上限时自动放宽(适用于急弯)
#define VISION_YAW_BOOST_MAX  60.0f       // 饱和解除后偏航上限(°)
#define VISION_YAW_I_BOOST    40.0f       // 饱和解除后积分上限(°)
#define SATURATE_HOLD_COUNT   3           // 连续饱和N次(50ms×N=150ms)后解除

extern uint8 task;
extern uint8 threshold_1;
extern uint8 threshold_2;
extern uint8 zhongxian_flag;

//extern uint8 block;
//extern int16 offset;
//extern uint8 cy_left;


extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 erzhi_image[LCDH][LCDW];
extern uint8 temp_image[LCDH][LCDW];
extern uint8 use_image[LCDH][LCDW];
//uint8 visited[LCDH][LCDH];   // 用于递归的访问标记数组

//extern int16 left_edge[LCDH], right_edge[LCDH];



void ImageProcess(void);      //图像处理
void eye_show (uint8 x,uint8 y);
void detect_white_track_edge(uint8 *src, uint16 w, uint16 h, int16 *left_edge, int16 *right_edge);//边界判断
void draw_center_line(int16 *left_edge, int16 *right_edge, uint16 h, uint8 gray_value);//根据边界画中线

void Get_Use_Image(void);                                                                       //压缩图像60*94
void er_zhi(uint8 *src, uint8 *dst, uint16 w, uint16 h, uint8 th);
void er_zhi_dynamic(uint8 *src, uint8 *dst, uint16 w, uint16 h);   //3区动态Otsu二值化(自适应光照)

void Median_Filter_3x3(uint8 *src, uint8 *dst, uint16 w, uint16 h);                              //3*3中值滤波
void Median_Filter_5x5(uint8 *src, uint8 *dst, uint16 w, uint16 h);                             //5*5中值滤波，去噪更强


//形态学图像处理
void Erode(uint8 *src, uint8 *dst, uint16 w, uint16 h);                 //腐蚀
void Dilate(uint8 *src, uint8 *dst, uint16 w, uint16 h);                //膨胀
void Open_Operate(uint8 *img, uint8 *tmp, uint16 w, uint16 h);          //开运算
void Close_Operate(uint8 *img, uint8 *tmp, uint16 w, uint16 h);         //闭运算


/**********************************自定义结构体*****************************************/
/************************************梯形*************************************/
//边界结构体（必须先定义）
typedef struct {
    int16 left_edge[LCDH];//左边界
    int16 right_edge[LCDH];//右边界
}boundary;


/************************************梯形*************************************/

/************************************台阶检测*************************************/
// 跳距等级(台阶在图像中的位置→实际距离)
typedef enum {
    JUMP_TOO_FAR = 0,      // 台阶太远(图像中上部), 不可跳
    JUMP_APPROACH,         // 接近中, 准备起跳
    JUMP_READY,            // 到达起跳点, 可以跳
    JUMP_TOO_CLOSE,        // 太近(台阶已在画面底部之外), 错过起跳窗口
} JumpDistance;

// 台阶检测结果(替代梯形检测, 用于三级跳台第一级台阶识别)
typedef struct {
    uint8 found;          // 是否找到台阶
    int center_x;         // 台阶中心x坐标
    int center_y;         // 台阶中心y坐标
    int top_y;            // 台阶顶部y(远处)
    int bottom_y;         // 台阶底部y(近处)
    int region_width;     // 台阶平均宽度(像素)
    int region_height;    // 台阶高度(像素行数)
    int confidence;       // 置信度(0-100)
    JumpDistance distance;// 跳距等级
    uint8 jump_now;       // 起跳信号(1=立即跳)
} StepResult;
/************************************台阶检测*************************************/

/********************************外部变量**************************************/
extern StepResult step_result;
extern boundary;

/**
 * @brief 中线x坐标均值与图像中心对比数据
 * 
 * center_compare[0] = 图像的中线x坐标 (LCDW/2)
 * center_compare[1] = 绘制出的中线的x坐标均值
 * center_compare[2] = 差值 = 图像中线x - 绘制中线x均值
 * 
 * 可通过逐飞助手虚拟示波器查看对比
 */
extern volatile float center_compare[3];
/********************************外部变量**************************************/

/**********************************自定义结构体*****************************************/
//梯形检测
//白色赛道边缘检测
void detect_white_track_edge(uint8 *src, uint16 w, uint16 h, int16 *left_edge, int16 *right_edge);
void jump_step(void);
StepResult detect_step_hybrid(uint8 *src, uint16 w, uint16 h); //融合检测(边缘+梯形验证)
void xunzhongxian(void);



#endif
