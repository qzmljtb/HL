#include "camera.h"

float vision_yaw_target = 0.0f;                 //视觉偏航角目标(°): P+I总输出
float vision_yaw_i = 0.0f;               //视觉偏航积分项(°): 累积中线偏差 消除稳态误差

uint8 task = 2;

uint8 base_image[MT9V03X_H][MT9V03X_W];
uint8 use_image[LCDH][LCDW];
uint8 erzhi_image[LCDH][LCDW];

uint8 temp_image[LCDH][LCDW];

uint8 threshold_1 = 75;
uint8 threshold_2 = 70;
uint8 zhongxian_flag = 1;

boundary bridge={0};
/*********************************图像处理************************************/
void ImageProcess(void)
{
    Get_Use_Image();//压缩
    if(task == 1)
    {
        roll_balance_enable = 1;
        Median_Filter_3x3(use_image[0],temp_image[0],LCDW,LCDH);//中值滤�?
        er_zhi(temp_image[0],erzhi_image[0],LCDW,LCDH,threshold_1);
        Open_Operate(erzhi_image[0], temp_image[0], LCDW, LCDH);                    // 开运算（去噪点�?
        Close_Operate(erzhi_image[0], temp_image[0], LCDW, LCDH);                   // 闭运算（填坑洞）
        memcpy(temp_image[0], erzhi_image[0], LCDW * LCDH);
        Erode(temp_image[0], erzhi_image[0], LCDW, LCDH);     
        detect_white_track_edge(erzhi_image[0], LCDW, LCDH, bridge.left_edge, bridge.right_edge);
        draw_center_line(bridge.left_edge, bridge.right_edge,LCDH, 128);
    }
    else if(task == 2)
    {
        roll_balance_enable = 0;
        Median_Filter_3x3(use_image[0],temp_image[0],LCDW,LCDH);
        er_zhi(temp_image[0],erzhi_image[0],LCDW,LCDH,threshold_2);
        //er_zhi_dynamic(temp_image[0],erzhi_image[0],LCDW,LCDH);
        Open_Operate(erzhi_image[0], temp_image[0], LCDW, LCDH);                    
        Close_Operate(erzhi_image[0], temp_image[0], LCDW, LCDH);                   
        memcpy(temp_image[0], erzhi_image[0], LCDW * LCDH);
        Erode(temp_image[0], erzhi_image[0], LCDW, LCDH);
        jump_step();
    }
    else if(task == 3)
    {     
        roll_balance_enable = 0;
        Median_Filter_3x3(use_image[0],temp_image[0],LCDW,LCDH);//中值滤�?
    }
    
    
    
}
/*********************************图像处理*************************************/

/*******************************图像预处�?*************************************/
void Get_Use_Image(void)//压缩图像60*94
{
    uint8 mode = 0;
    uint16 i = 0, j = 0, row = 0, line = 0;
    if(mode == 0)
    {
        for (i = 0; i < MT9V03X_H; i += 2)          //�?120 / 2  = 60�?
        {
            for (j = 0; j < MT9V03X_W; j += 2)     //�?88 / 2  = 80�?
            {
                use_image[row][line] = base_image[i][j]; //mt9v03x_image[i][j]
                line++;
            }
             line = 0;
             row++;
        }
    }
}


void er_zhi(uint8 *src, uint8 *dst, uint16 w, uint16 h, uint8 th)
{
    uint8 i,j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            if(src[i * w + j] < th)
                dst[i * w + j] = 0;
            else
                dst[i * w + j] = 255;
        }
    }
}


//============================================================================
// 3区动态Otsu二值化 — 赛道光照自适应
// 将图像垂直分为 远/中/近 三个区域，每区独立计算Otsu阈值
// 解决远近光照不均导致的固定阈值二值化失效问题
//============================================================================

#define ZONE_FAR_ROWS   20      //远处区域行数(0~19, 赛道消失点附近)
#define ZONE_NEAR_ROWS  20      //近处区域行数(40~59, 车轮前方)
// 中间区域 = 20~39, 自动取剩余行

#define THRESH_MIN      40      //阈值下限: 防止全黑区域误判
#define THRESH_MAX      180     //阈值上限: 防止全白区域误判

// 单区Otsu阈值计算(1D指针版, 匹配当前图像存储格式)
static uint8 calc_zone_otsu(uint8 *pic, uint16 w, uint16 y_start, uint16 y_end)
{
    uint32 pixel_dist[256] = {0};
    uint16 total_px = (y_end - y_start) * w;

    // 统计直方图
    for (uint16 y = y_start; y < y_end; y++)
    {
        uint8 *row = &pic[y * w];
        for (uint16 x = 0; x < w; x++)
        {
            pixel_dist[row[x]]++;
        }
    }

    // 总灰度加权和
    float total_val = 0.0f;
    for (uint16 k = 0; k < 256; k++)
    {
        total_val += (float)(k * pixel_dist[k]);
    }

    // 遍历阈值找最大类间方差
    float acc_b = 0.0f, max_variance = 0.0f;
    uint32 weight_bg = 0;
    uint8 optimal_thresh = 128;

    for (uint16 k = 0; k < 256; k++)
    {
        weight_bg += pixel_dist[k];
        if (weight_bg == 0) continue;

        uint32 weight_fg = total_px - weight_bg;
        if (weight_fg == 0) break;

        acc_b += (float)(k * pixel_dist[k]);
        float mean_bg = acc_b / (float)weight_bg;
        float mean_fg = (total_val - acc_b) / (float)weight_fg;
        float diff = mean_bg - mean_fg;
        float var_between = (float)weight_bg * (float)weight_fg * diff * diff;

        if (var_between > max_variance)
        {
            max_variance = var_between;
            optimal_thresh = (uint8)k;
        }
    }
    return optimal_thresh;
}

/**
 * @brief 3区动态Otsu二值化 — 替代固定阈值 er_zhi
 * @param src  输入灰度图 (1D指针, 尺寸 w×h)
 * @param dst  输出二值图 (0=黑/赛道, 255=白/背景)
 * @param w    图像宽度
 * @param h    图像高度 (需 ≥ ZONE_FAR_ROWS + ZONE_NEAR_ROWS)
 *
 * 用法: 在 ImageProcess() 中将 er_zhi(...) 替换为 er_zhi_dynamic(...)
 */
void er_zhi_dynamic(uint8 *src, uint8 *dst, uint16 w, uint16 h)
{
    static uint8 t_far  = 128;   //远处阈值(初始128, 动态更新)
    static uint8 t_mid  = 128;   //中段阈值
    static uint8 t_near = 128;   //近处阈值
    static uint8 refresh_tick = 0;

    // 每帧重算Otsu (20Hz刷新率, 够用)
    if (++refresh_tick >= 1)
    {
        refresh_tick = 0;

        uint16 mid_start = ZONE_FAR_ROWS;
        uint16 near_start = h - ZONE_NEAR_ROWS;

        t_far  = calc_zone_otsu(src, w, 0, mid_start);
        t_mid  = calc_zone_otsu(src, w, mid_start, near_start);
        t_near = calc_zone_otsu(src, w, near_start, h);

        // 限幅: 防止均匀区域Otsu跑偏
        if (t_far  < THRESH_MIN) t_far  = THRESH_MIN;
        if (t_far  > THRESH_MAX) t_far  = THRESH_MAX;
        if (t_mid  < THRESH_MIN) t_mid  = THRESH_MIN;
        if (t_mid  > THRESH_MAX) t_mid  = THRESH_MAX;
        if (t_near < THRESH_MIN) t_near = THRESH_MIN;
        if (t_near > THRESH_MAX) t_near = THRESH_MAX;
    }

    // 逐行二值化(三区分别用各自阈值)
    uint16 mid_start = ZONE_FAR_ROWS;
    uint16 near_start = h - ZONE_NEAR_ROWS;

    for (uint16 y = 0; y < h; y++)
    {
        uint8 active_t;
        if (y < mid_start)
            active_t = t_far;
        else if (y < near_start)
            active_t = t_mid;
        else
            active_t = t_near;

        uint8 *src_row = &src[y * w];
        uint8 *dst_row = &dst[y * w];
        for (uint16 x = 0; x < w; x++)
        {
            // 灰度 > 阈值 → 白色(背景/赛道)
            dst_row[x] = (src_row[x] > active_t) ? 255 : 0;
        }
    }
}


/*********************************滤波�?***************************************/
//  3*3 中值滤波，通用性强
void Median_Filter_3x3(uint8 *src, uint8 *dst, uint16 w, uint16 h)
{
    uint16 i,j;
    uint8 buf[9];
    for(i = 1; i < h-1; i++)
    {
        for(j = 1; j < w-1; j++)
        {
            // 取出3x3窗口9个像�?
            buf[0] = src[(i-1)*w + j-1];
            buf[1] = src[(i-1)*w + j];
            buf[2] = src[(i-1)*w + j+1];
            buf[3] = src[i*w + j-1];
            buf[4] = src[i*w + j];
            buf[5] = src[i*w + j+1];
            buf[6] = src[(i+1)*w + j-1];
            buf[7] = src[(i+1)*w + j];
            buf[8] = src[(i+1)*w + j+1];
            
            // 用冒泡排序取中�?
            for(int a=0;a<8;a++)
            {
                for(int b=a+1;b<9;b++)
                {
                    if(buf[a] > buf[b])
                    {
                        uint8 t = buf[a];
                        buf[a] = buf[b];
                        buf[b] = t;
                    }
                }
            }
            dst[i*w + j] = buf[4]; // 取中间�?
        }
    }
    // 边界拷贝
    for(i = 0; i < h; i++)
    {
        dst[i*w] = src[i*w];
        dst[i*w + w-1] = src[i*w + w-1];
    }
    for(j = 0; j < w; j++)
    {
        dst[j] = src[j];
        dst[(h-1)*w + j] = src[(h-1)*w + j];
    }
}

//  5*5 中值滤波，去噪更强，但边缘细节保留较差
void Median_Filter_5x5(uint8 *src, uint8 *dst, uint16 w, uint16 h)
{
    uint16 i,j;
    uint8 buf[25];
    for(i = 2; i < h-2; i++)
    {
        for(j = 2; j < w-2; j++)
        {
            // 取出5x5窗口25个像�?
            uint8 idx = 0;
            for(int dy = -2; dy <= 2; dy++)
            {
                for(int dx = -2; dx <= 2; dx++)
                {
                    buf[idx++] = src[(i+dy)*w + (j+dx)];
                }
            }
            
            // 冒泡排序取中值，�?3�?
            for(int a=0;a<24;a++)
            {
                for(int b=a+1;b<25;b++)
                {
                    if(buf[a] > buf[b])
                    {
                        uint8 t = buf[a];
                        buf[a] = buf[b];
                        buf[b] = t;
                    }
                }
            }
            dst[i*w + j] = buf[12]; // 取中间值，�?3个（0-indexed=12�?
        }
    }
    // 边界拷贝，上下左�?�?列直接复�?
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            if(i < 2 || i >= h-2 || j < 2 || j >= w-2)
            {
                dst[i*w + j] = src[i*w + j];
            }
        }
    }
}


/**********************************形态学运算***********************************/
// 形态学腐蚀�?x3 全结构元�?
void Erode(uint8 *src, uint8 *dst, uint16 w, uint16 h)
{
    uint16 i,j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            // 边界像素，只取有效区域判断，防止越界
            uint8 flag = 1;
            for(int dy = -1; dy <= 1; dy++)
            {
                for(int dx = -1; dx <= 1; dx++)
                {
                    int ny = i + dy;
                    int nx = j + dx;
                    // 越界像素视为255，不影响腐蚀，防止边界被腐蚀成黑�?
                    if(ny >= 0 && ny < h && nx >= 0 && nx < w)
                    {
                        if(!src[ny * w + nx])
                        {
                            flag = 0;
                            break;
                        }
                    }
                }
                if(!flag) break;
            }
            dst[i*w + j] = flag ? 255 : 0;
        }
    }
}

// 形态学膨胀�?x3 全结构元�?
void Dilate(uint8 *src, uint8 *dst, uint16 w, uint16 h)
{
    uint16 i,j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            uint8 flag = 0;
            for(int dy = -1; dy <= 1; dy++)
            {
                for(int dx = -1; dx <= 1; dx++)
                {
                    int ny = i + dy;
                    int nx = j + dx;
                    // 越界像素视为255，白色，防止边界膨胀成黑�?
                    if(ny >= 0 && ny < h && nx >= 0 && nx < w)
                    {
                        if(src[ny * w + nx])
                        {
                            flag = 1;
                            break;
                        }
                    }
                    else
                    {
                        // 越界像素视为白色，保证边界被膨胀不影响黑�?
                        flag = 1;
                        break;
                    }
                }
                if(flag) break;
            }
            dst[i*w + j] = flag ? 255 : 0;
        }
    }
}

// 开运算 = 腐蚀 + 膨胀（去噪点�?
// 输入 img，结果写�?img，tmp 为临时缓冲区
void Open_Operate(uint8 *img, uint8 *tmp, uint16 w, uint16 h)
{
    Erode(img, tmp, w, h);
    Dilate(tmp, img, w, h);
}

// 闭运�?= 膨胀 + 腐蚀（填坑洞�?
// 输入 img，结果写�?img，tmp 为临时缓冲区
void Close_Operate(uint8 *img, uint8 *tmp, uint16 w, uint16 h)
{
    Dilate(img, tmp, w, h);
    Erode(tmp, img, w, h);   // 注意：是 Erode(tmp, img, w, h)
}

/*******************************白色赛道边缘检�?*******************************/

/**
 * @brief 白色赛道边缘检�?
 * 
 * 用于白色赛道，二值化后白�?255，黑�?0，赛道为白色
 * 分别扫描"�?>�?跳变（左边界）和"�?>�?跳变（右边界�?
 * 
 * @param src         二值图像指�?(erzhi_image)
 * @param w           图像宽度
 * @param h           图像高度
 * @param left_edge   输出每行左边界x坐标数组（未找到�?1�?
 * @param right_edge  输出每行右边界x坐标数组（未找到�?1�?
 */
void detect_white_track_edge(uint8 *src, uint16 w, uint16 h, int16 *left_edge, int16 *right_edge)
{
    uint16 skip_top = 5;       // 跳过顶部噪点行(0~4)
    uint16 x, y;

    // 每行统计: min_x=该行最小跳变列, max_x=该行最大跳变列, count=跳变点数
    int16  min_x[LCDH];
    int16  max_x[LCDH];
    uint8  count[LCDH];

    // 初始化
    for (y = 0; y < h; y++)
    {
        left_edge[y] = -1;
        right_edge[y] = -1;
        min_x[y] = (int16)(w);
        max_x[y] = -1;
        count[y] = 0;
    }

    // 列扫描: 左右交替同时向中间推进
    uint16 left_col  = 0;
    uint16 right_col = (uint16)(w - 1);
    uint8  scan_left = 1;      // 1=扫左列, 0=扫右列

    while (left_col <= right_col)
    {
        // 交替选择当前列
        if (scan_left)
        {
            x = left_col;
            left_col++;
        }
        else
        {
            x = right_col;
            right_col--;
        }
        scan_left = !scan_left;

        // 该列自下而上扫描(y=H-1 到 y=skip_top)
        // 分别记录黑→白(赛道起点)和白→黑(赛道终点)两种跳变
        uint8 found_w2b = 0;   // 白→黑已找到(赛道终点/离开赛道)
        uint8 found_b2w = 0;   // 黑→白已找到(赛道起点/进入赛道)

        for (y = (uint16)(h - 1); y >= skip_top; y--)
        {
            uint8 cur  = src[ y      * w + x];
            uint8 prev = src[(y - 1) * w + x];

            // 白→黑(255→0): 离开赛道, 对应赛道终点/右边界
            if (!found_w2b && cur == 255 && prev == 0)
            {
                found_w2b = 1;
                count[y]++;
                if ((int16)x < min_x[y]) min_x[y] = (int16)x;
                if ((int16)x > max_x[y]) max_x[y] = (int16)x;
            }
            // 黑→白(0→255): 进入赛道, 对应赛道起点/左边界
            else if (!found_b2w && cur == 0 && prev == 255)
            {
                found_b2w = 1;
                count[y]++;
                if ((int16)x < min_x[y]) min_x[y] = (int16)x;
                if ((int16)x > max_x[y]) max_x[y] = (int16)x;
            }

            // 两种跳变都找到, 停止该列扫描
            if (found_w2b && found_b2w) break;
        }
    }

    // 汇总输出: 同行 ≥2 个跳变点 → 取 min/max 为左右边界
    for (y = 0; y < h; y++)
    {
        if (count[y] >= 2)
        {
            left_edge[y]  = min_x[y];
            right_edge[y] = max_x[y];
        }
        // else: 保持 -1 (无效行)
    }
}

/*******************************图像绘制**************************************/

void eye_show (uint8 x,uint8 y)//显示十字
{
    // 边界保护
    if(x >= LCDW || y >= LCDH) return;
    
    // 画竖线，在x列画�?
    for(uint8 i = 0; i < LCDH; i++)
    {
        erzhi_image[i][x] = 0;  // 黑色填充
    }
    // 画横线，在y行画�?
    for(uint8 i = 0; i < LCDW; i++)
    {
        erzhi_image[y][i] = 0;  // 黑色填充
    }
}

/**
 * @brief 中线x坐标均值与图像中心对比数据
 * 
 * center_compare[0] = 图像的中线x坐标 (LCDW/2)
 * center_compare[1] = 绘制出的中线的x坐标均�?
 * center_compare[2] = 差�?= 图像中线x - 绘制中线x均�?
 * 
 * 可通过逐飞助手虚拟示波器查看对�?
 */
volatile float center_compare[3] = {0};

/**
 * @brief 根据左右边界计算中线并绘制，同时计算中线x坐标均值并与图像中心对�?
 * 
 * 中线 = (left_edge + right_edge) / 2
 * 只有左右边界都有效时才画中线
 * 如果左右边界突然跳变较大（超过阈值），则沿用上一行的中线位置
 * 
 * @param left_edge   左边界x坐标数组
 * @param right_edge  右边界x坐标数组
 * @param h           图像高度
 * @param gray_value  中线绘制的灰度�?
 */
void draw_center_line(int16 *left_edge, int16 *right_edge, uint16 h, uint8 gray_value)
{
    int16 prev_center_x = -1;
    int16 prev_left = -1;
    int16 prev_right = -1;
    int16 jump_threshold = 8;
    int32 center_sum = 0;       // 中线x坐标累加
    uint16 center_count = 0;    // 有效中线点数计数

    // 忽略顶部和底部各 5 行（图像边缘噪点多，边界检测不可靠）
    uint16 skip_bottom = 5;     // 底部忽略行数
    uint16 skip_top = 5;        // 顶部忽略行数
    int start_y = (int)h - 1 - (int)skip_bottom;  // 扫描起始行（跳过底部）
    int end_y = (int)skip_top;                     // 扫描结束行（跳过顶部）

    for(int y = start_y; y >= end_y; y--)
    {
        int16 center_x = -1;
        int16 cur_left = left_edge[y];
        int16 cur_right = right_edge[y];

        if(cur_left >= 0 && cur_right >= 0)
        {
            uint8 left_jump = 0, right_jump = 0;
            if(prev_left >= 0 && prev_right >= 0)
            {
                int16 left_diff = cur_left - prev_left;
                int16 right_diff = cur_right - prev_right;
                if(left_diff < 0) left_diff = -left_diff;
                if(right_diff < 0) right_diff = -right_diff;
                if(left_diff > jump_threshold) left_jump = 1;
                if(right_diff > jump_threshold) right_jump = 1;
            }

            if(left_jump || right_jump)
            {
                // 边界突变：沿用前一行中心保持连续性
                center_x = (prev_center_x >= 0) ? prev_center_x : (cur_left + cur_right) / 2;
            }
            else
            {
                center_x = (cur_left + cur_right) / 2;
            }

            // 始终更新 prev 边界为当前有效值，避免连锁误判跳变
            prev_left = cur_left;
            prev_right = cur_right;
        }
        else
        {
            // 无效行（无赛道）：不延伸中线，该行跳过
            center_x = -1;
        }

        if(center_x >= 0 && center_x < LCDW && erzhi_image[y][center_x] == 255)
        {
            use_image[y][center_x] = gray_value;
            erzhi_image[y][center_x] = 128;
            prev_center_x = center_x;

            // 累加有效中线的x坐标
            center_sum += center_x;
            center_count++;
        }
    }

    // 计算中线x坐标均值，并与图像中心对比
    center_compare[0] = (float)LCDW / 2.0f;  // 图像的中线x坐标 (94/2 = 47)
    if(center_count > 0)
    {
        center_compare[1] = (float)center_sum / center_count;  // 绘制出的中线x均值
    }
    else
    {
        center_compare[1] = -1;  // 无有效中线
    }
    center_compare[2] = center_compare[0] - center_compare[1];  // 差值 = 图像中线x - 绘制中线x均值
}



//============================================================================
// 台阶检测 — 底部白色区域定位（三级跳台第一级台阶识别）
// 原理: 自下而上单遍扫描, 找到第一个(最低的)连续白色区域
//       白色计数 + 边界检测 + 区域识别 合为一次遍历
//============================================================================

#define STEP_WHITE_RATIO     0.45f    // 行白色像素比例阈值(折中: 原0.45, 过滤细白线)
#define STEP_MIN_HEIGHT      6        // 台阶最小高度(行)(折中: 原6, 过滤矮白块)
#define STEP_MIN_WIDTH       12       // 台阶最小宽度(像素)(折中: 原12, 过滤窄白条)
#define STEP_MIN_CENTER_Y    22       // 台阶中心最低y坐标(必须在图像下半部)
#define STEP_TEMPORAL_FRAMES 1        // 连续确认帧数(高速适配: 原5→2, 防高速时来不及确认)
#define STEP_LOST_FRAMES     2        // 连续丢失帧数(超过此值才复位, 防偶发噪声)
#define STEP_EMA_ALPHA       0.3f     // EMA平滑系数(0~1, 越小越平滑, 0.3≈70%旧值+30%新值)
#define STEP_SKIP_BOTTOM     3        // 跳过最底部N行(车体/噪点)
#define STEP_GAP_CHECK_ROWS  5        // 间隙检查行数
#define STEP_MAX_DY          25       // 帧间最大y位移(高速适配: 原15→25, 防dy超限清零confirm_cnt)

// 起跳距离阈值(台阶底部y坐标 → 实际距离映射, 需根据摄像头安装角度标定)
#define JUMP_BOTTOM_APPROACH      0  // bottom_y ≥ 此值 → 接近中, 准备起跳
#define JUMP_BOTTOM_READY_BEGIN   0  // 起跳窗口下限: bottom_y ∈ [BEGIN, END) 为起跳区间
#define JUMP_BOTTOM_READY_END     60  // 起跳窗口上限: 超此值视为错过最佳起跳点
#define JUMP_BOTTOM_TOO_CLOSE     60  // bottom_y ≥ 此值 → 太近, 错过起跳窗口
#define JUMP_NOW_HOLD_FRAMES       5  // jump_now=1 保持帧数(防止单帧太快被错过)

StepResult step_result = {0};
static JumpDistance last_jump_distance = JUMP_TOO_FAR;  // 跳距滞回状态(台阶丢失时由jump_step复位)
static uint8 jump_now_fired = 0;                       // 起跳窗口内已触发标志(每个窗口周期仅触发一次)
static uint8 jump_now_hold_cnt = 0;                     // jump_now保持计数(>0时持续输出1)
#define STEP_LOST_RESET_FRAMES 5   // 连续丢失N帧后才复位last_jump_distance(防偶发丢帧)

//============================================================================
// 台阶检测Hybrid — 边缘跳变定位 + 梯形形状验证
// 融合 detect_first_step 的稳定性 与 detect_isosceles_trapezoid 的边缘鲁棒性
//============================================================================
StepResult detect_step_hybrid(uint8 *src, uint16 w, uint16 h)
{
    StepResult result = {0};
    int scan_limit = (int)h - 1 - STEP_SKIP_BOTTOM;

    // ---- Phase 1&2 合并: 单次自下而上扫描, 同时完成边缘检测+属性累加 ----
    int region_bottom = -1, region_top = -1;
    int32 width_sum = 0, center_x_sum = 0;
    int valid_rows = 0, top_width = 0, bottom_width = 0;
    int consec_invalid = 0;  // 连续无效行计数(BugFix: 原逻辑误判未处理行)

    for (int y = scan_limit; y >= 0; y--)
    {
        uint8 *row = &src[y * w];
        int16 l = -1, r = -1;

        // 单次左→右扫描: 找第一个和最后一个白像素(等价于边缘, 且更鲁棒)
        for (int x = 0; x < (int)w; x++)
        {
            if (row[x] > 0)
            {
                if (l < 0) l = (int16)x;   // 首个白像素 = 左边缘
                r = (int16)x;               // 持续更新 = 右边缘
            }
        }

        int width = (l >= 0 && r > l) ? (r - l + 1) : 0;

        if (width >= STEP_MIN_WIDTH)
        {
            consec_invalid = 0;             // 有效行, 清零连续无效计数

            if (region_bottom < 0)
            {
                region_bottom = y;
                region_top   = y;
                bottom_width = width;
                top_width    = width;
            }
            else
            {
                region_top = y;
                top_width  = width;         // 持续更新顶部宽度
            }

            width_sum    += width;
            center_x_sum += (l + r) / 2;
            valid_rows++;
        }
        else if (region_bottom >= 0)
        {
            // 允许1行间断, 连续≥2行无效才终止(BugFix: 用计数器替代数组回查)
            consec_invalid++;
            if (consec_invalid >= 2)
                break;
        }
    }

    if (region_bottom < 0 || region_top < 0) return result;

    // ---- 基本校验 ----
    int region_height = region_bottom - region_top + 1;
    if (region_height < STEP_MIN_HEIGHT || valid_rows < STEP_MIN_HEIGHT / 2)
        return result;

    int avg_width = (valid_rows > 0) ? (int)(width_sum / valid_rows) : 0;
    int center_x  = (valid_rows > 0) ? (int)(center_x_sum / valid_rows) : 0;
    int center_y  = (region_top + region_bottom) / 2;

    if (avg_width < STEP_MIN_WIDTH || center_y < STEP_MIN_CENTER_Y)
        return result;

    // ---- Phase 3: 梯形形状验证(收紧: 底宽≥顶宽×0.9, 真台阶不缩窄) ----
    uint8 shape_ok = 0;
    float width_ratio = (top_width > 0) ? (float)bottom_width / (float)top_width : 1.0f;
    if (width_ratio >= 0.9f && width_ratio <= 2.5f)
        shape_ok = 1;

    // ---- Phase 4: 间隙检查 ----
    uint8 has_gap_above = 0;
    int gap_start = region_top - 1;
    int gap_end   = region_top - STEP_GAP_CHECK_ROWS;
    if (gap_end < 0) gap_end = 0;
    if (gap_start >= 0)
    {
        for (int y = gap_start; y >= gap_end; y--)
        {
            uint8 *row = &src[y * w];
            uint16 black_cnt = 0;
            for (int x = 0; x < (int)w; x++)
                if (row[x] == 0) black_cnt++;
            if ((float)black_cnt / (float)w > 0.75f)
                { has_gap_above = 1; break; }
        }
    }

    // ---- Phase 5: 置信度评估 ----
    int confidence = 40;
    if (region_height >= 12) confidence += 15;
    else if (region_height >= 8) confidence += 8;
    if (avg_width >= 20 && avg_width <= 70) confidence += 15;
    else if (avg_width >= 15) confidence += 8;
    if (shape_ok)      confidence += 10;
    if (has_gap_above) confidence += 15;
    if (center_y >= 35) confidence += 10;
    else if (center_y >= 28) confidence += 5;
    if (confidence > 100) confidence = 100;

    // ---- Phase 6: 时域滤波(EMA+丢失容错) ----
    static uint8 confirm_cnt = 0;
    static uint8 lost_cnt    = 0;
    static int last_center_x = -1, last_center_y = -1;
    static float ema_center_x = 0, ema_center_y = 0, ema_bottom_y = 0;
    static uint8 ema_inited = 0;

    if (confidence >= 40)
    {
        lost_cnt = 0;
        if (!ema_inited)
        {
            ema_center_x = (float)center_x;
            ema_center_y = (float)center_y;
            ema_bottom_y = (float)region_bottom;
            ema_inited   = 1;
        }
        else
        {
            ema_center_x = STEP_EMA_ALPHA * center_x      + (1.0f - STEP_EMA_ALPHA) * ema_center_x;
            ema_center_y = STEP_EMA_ALPHA * center_y      + (1.0f - STEP_EMA_ALPHA) * ema_center_y;
            ema_bottom_y = STEP_EMA_ALPHA * region_bottom + (1.0f - STEP_EMA_ALPHA) * ema_bottom_y;
        }

        if (last_center_x >= 0)
        {
            int dx = center_x - last_center_x;  if (dx < 0) dx = -dx;
            int dy = center_y - last_center_y;  int dy_abs = (dy < 0) ? -dy : dy;
            if (dx <= STEP_MAX_DY && dy_abs <= STEP_MAX_DY)
            {
                if (confirm_cnt < STEP_TEMPORAL_FRAMES) confirm_cnt++;
            }
            else { confirm_cnt = 0; }
        }
        last_center_x = center_x;
        last_center_y = center_y;
    }
    else
    {
        if (confirm_cnt >= STEP_TEMPORAL_FRAMES && lost_cnt < STEP_LOST_FRAMES)
            { lost_cnt++; }
        else
        {
            confirm_cnt = 0; lost_cnt = 0;
            last_center_x = -1; last_center_y = -1;
            ema_inited = 0;
        }
    }

    if (confirm_cnt < STEP_TEMPORAL_FRAMES)
        return result;

    // EMA平滑输出
    center_x      = (int)(ema_center_x + 0.5f);
    center_y      = (int)(ema_center_y + 0.5f);
    region_bottom = (int)(ema_bottom_y + 0.5f);

    result.found         = 1;
    result.center_x      = center_x;
    result.center_y      = center_y;
    result.top_y         = region_top;
    result.bottom_y      = region_bottom;
    result.region_width  = avg_width;
    result.region_height = region_height;
    result.confidence    = confidence;

    // ---- Phase 7: 跳距判定(区间窗口 + 单次触发 + 多帧保持) ----
    // distance分级: READY = [READY_BEGIN, TOO_CLOSE), 宽区间兼容直接进入高值的情况
    if (region_bottom >= JUMP_BOTTOM_TOO_CLOSE)
        result.distance = JUMP_TOO_CLOSE;
    else if (region_bottom >= JUMP_BOTTOM_READY_BEGIN)
        result.distance = JUMP_READY;
    else if (region_bottom >= JUMP_BOTTOM_APPROACH)
        result.distance = JUMP_APPROACH;
    else
        result.distance = JUMP_TOO_FAR;

    // 滞回: 距离状态只进不退(台阶不可能变远)
    if (result.distance < last_jump_distance)
        result.distance = last_jump_distance;
    else
        last_jump_distance = result.distance;

    // 起跳信号: 首次进入READY区间时锁存, 保持 JUMP_NOW_HOLD_FRAMES 帧
    // (避免单帧太快导致下游未来得及响应, 也防止速度过快跳过窗口)
    if (result.distance == JUMP_READY && !jump_now_fired
        && region_bottom < JUMP_BOTTOM_READY_END)
    {
        jump_now_fired = 1;
        jump_now_hold_cnt = JUMP_NOW_HOLD_FRAMES;  // 装填保持计数
    }

    if (jump_now_hold_cnt > 0)
    {
        result.jump_now = 1;
        jump_now_hold_cnt--;
    }
    else
    {
        result.jump_now = 0;
    }

    return result;
}

void jump_step(void)
{
    static uint8 lost_reset_cnt = 0;   // 连续信号丢失计数(防偶发丢帧导致跳距复位)

    // 台阶检测: 必须先于中线绘制(中线会修改 erzhi_image, 干扰白色像素统计)
    step_result = detect_step_hybrid(erzhi_image[0], LCDW, LCDH);

    // 中线绘制: 用于弯道循迹, 防止跑偏
    detect_white_track_edge(erzhi_image[0], LCDW, LCDH, bridge.left_edge, bridge.right_edge);
    draw_center_line(bridge.left_edge, bridge.right_edge, LCDH, 128);

    if (step_result.found)
    {
        lost_reset_cnt = 0;   // 有信号, 清零丢失计数

        for (int x = 0; x < LCDW; x++)
        {
            if (step_result.top_y >= 0 && step_result.top_y < LCDH)
                erzhi_image[step_result.top_y][x] = 128;
            if (step_result.bottom_y >= 0 && step_result.bottom_y < LCDH)
                erzhi_image[step_result.bottom_y][x] = 128;
        }
    }
    else
    {
        // 连续丢失超过阈值才复位, 防止偶发噪声导致跳距状态丢失
        lost_reset_cnt++;
        if (lost_reset_cnt >= STEP_LOST_RESET_FRAMES)
        {
            last_jump_distance = JUMP_TOO_FAR;
            jump_now_fired = 0;          // 重置起跳触发标志
            jump_now_hold_cnt = 0;       // 清零保持计数
            lost_reset_cnt = 0;
        }
    }

    // 起跳检查放在 if(found) 外面:
    // jump_now_hold_cnt>0 兜底 — 台阶暂丢时 hold 计数不递减(Phase7被跳过),
    // 但 jump_now 会因提前 return 而清零, 用 hold_cnt 补充保证信号不丢
    static uint8 a = 0;
    if ((step_result.jump_now || jump_now_hold_cnt > 0)
        && go_flag >= 3
        && euler_angle.pitch >= -30 && euler_angle.pitch <= 30
        && euler_angle.roll  >= -35 && euler_angle.roll  <= 35
        && a < 1)
        {
            a++;
            jump_trigger();
        }
       
}

void xunzhongxian(void)
{
    // === 视觉偏航角目标更新(P+I结构 + 自适应饱和解除) ===
        // center_compare[2] = 图像中线(47) - 赛道中线均值
        //   正值(+) → 赛道偏左 → 车偏右 → 需增加偏航目标左转
        //   负值(-) → 赛道偏右 → 车偏左 → 需减小偏航目标右转
        if (zhongxian_flag == 1 && center_compare[1] >= 0)       //转向使能 且 有效中线
        {
            static uint8 saturate_cnt = 0;                 //连续饱和计数
            static int8  saturate_dir = 0;                  //饱和方向: +1=正, -1=负, 0=未饱和
            float error = center_compare[2];               //当前像素偏差

            // P项: 偏差立即映射为角度 → 快速响应弯道
            float vision_yaw_p = error * VISION_YAW_KP;

            // I项: 缓慢累积消除稳态误差 → 保证直道不偏
            vision_yaw_i += error * VISION_YAW_KI;

            // === 自适应饱和解除: 目标持续撞上限 → 临时抬升上限 ===
            float i_max   = VISION_YAW_I_MAX;              //当前积分上限
            float yaw_max = VISION_YAW_MAX;                //当前偏航上限

            float raw_target = vision_yaw_p + vision_yaw_i;
            if (raw_target > VISION_YAW_MAX && error > 0)  //正向饱和 且 偏差方向一致
            {
                if (saturate_dir == 1) saturate_cnt++;
                else { saturate_dir = 1; saturate_cnt = 1; }

                if (saturate_cnt >= SATURATE_HOLD_COUNT)   //持续150ms → 解除限制
                {
                    i_max   = VISION_YAW_I_BOOST;
                    yaw_max = VISION_YAW_BOOST_MAX;
                }
            }
            else if (raw_target < -VISION_YAW_MAX && error < 0) //反向饱和
            {
                if (saturate_dir == -1) saturate_cnt++;
                else { saturate_dir = -1; saturate_cnt = 1; }

                if (saturate_cnt >= SATURATE_HOLD_COUNT)
                {
                    i_max   = VISION_YAW_I_BOOST;
                    yaw_max = VISION_YAW_BOOST_MAX;
                }
            }
            else                                           //未饱和 或 偏差已反向 → 复位
            {
                saturate_cnt = 0;
                saturate_dir = 0;
            }

            // 限幅(使用动态上限)
            if (vision_yaw_i >  i_max)   vision_yaw_i =  i_max;
            if (vision_yaw_i < -i_max)   vision_yaw_i = -i_max;

            vision_yaw_target = vision_yaw_p + vision_yaw_i;
            if (vision_yaw_target >  yaw_max) vision_yaw_target =  yaw_max;
            if (vision_yaw_target < -yaw_max) vision_yaw_target = -yaw_max;
        }
        else if (zhongxian_flag == 1)                             //转向使能 但 中线丢失 → 紧急停车
        {
            //run_speed = 0;                                 //目标速度清零 电机停转
            vision_yaw_i = 0.0f;                           //积分清零 避免恢复时突变
        }
        else
        {
          vision_yaw_target = 0;
        }

}
