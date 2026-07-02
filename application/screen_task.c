/**
 * @file screen_task.c
 * @brief LCD 导航路径+状态单页面显示实现 (10Hz)
 *
 * 参考 E03_uart_receiver_demo screen.c:
 *   - screen_display_map() → draw_map_grid() 一次性网格
 *   - display_route()       → 每周期单点累积不清屏
 *   - draw_big_point()      → 3×3 方块 + 白心
 *   - standardizing_x/y()   → 世界坐标→屏幕像素, 边界钳位
 */
#include "screen_task.h"
#include "cmsis_os.h"
#include "lcd.h"
#include "key.h"
#include "robot_def.h"

/* ==================================================================
 *  地图参数
 * ================================================================== */
#define MAP_X        0
#define MAP_Y        0
#define MAP_W        280
#define MAP_H        176

/* 世界坐标半轴范围 (参考 x_scale / y_scale_forward / y_scale_backward 约定) */
#define SCALE_X_HALF     6.0f    /* X 半轴 6m → 总宽 12m */
#define SCALE_Y_FORWARD  5.0f    /* Y 前向半轴 5m */
#define SCALE_Y_BACKWARD 2.0f    /* Y 后向半轴 2m */
#define SCALE_Y_TOTAL    (SCALE_Y_FORWARD + SCALE_Y_BACKWARD)  /* 7m */

/* ==================================================================
 *  状态区参数
 * ================================================================== */
#define STATUS_Y    176         /* 状态区起始 Y */
#define ROW1_Y      179         /* 第 1 行 YAW / X / Y */
#define ROW2_Y      195         /* 第 2 行 TX / TY / VIS / MD */
#define ROW3_Y      211         /* 第 3 行 BAD */

/* ==================================================================
 *  模块指针 (由 ScreenNavInit 传入)
 * ================================================================== */
static OpticalFlowInstance *screen_flow;
static attitude_t          *screen_ins;
static Vision_Recv_s       *screen_vision;
static RC_ctrl_t           *screen_rc;

/* ==================================================================
 *  坐标变换 (参考 standardizing_x / standardizing_y)
 * ================================================================== */

/**
 * @brief  世界 X 坐标 → 屏幕像素 X
 * @note   world_x ∈ [-SCALE_X_HALF, +SCALE_X_HALF] → px ∈ [0, MAP_W-1]
 */
static int standardizing_x(float world_x)
{
    int px;
    px = (int)((float)MAP_W * world_x / SCALE_X_HALF / 2.0f);
    px += MAP_W / 2;
    if (px < 0)       px = 0;
    if (px >= MAP_W)  px = MAP_W - 1;
    return px;
}

/**
 * @brief  世界 Y 坐标 → 屏幕像素 Y
 * @note   world_y ∈ [-SCALE_Y_BACKWARD, +SCALE_Y_FORWARD] → px ∈ [0, MAP_H-1]
 *         屏幕上方 = 世界前向 (正向 Y)
 */
static int standardizing_y(float world_y)
{
    int px;
    px = (int)((float)MAP_H * world_y / SCALE_Y_TOTAL);
    px = (int)((float)MAP_H * SCALE_Y_FORWARD / SCALE_Y_TOTAL) - px;
    if (px < 0)       px = 0;
    if (px >= MAP_H)  px = MAP_H - 1;
    return px;
}

/* ==================================================================
 *  大点绘制 (3×3 方块 + 白心, 参考 draw_big_point)
 * ================================================================== */
static void draw_big_point(int x, int y, uint16_t color)
{
    /* 钳位防止 3×3 方块溢出地图边界 */
    if (x < 2)       x = 2;
    if (x >= MAP_W - 2)  x = MAP_W - 3;
    if (y < 2)       y = 2;
    if (y >= MAP_H - 2)  y = MAP_H - 3;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            LCD_DrawPoint(x + i, y + j, color);
        }
    }
    LCD_DrawPoint(x, y, WHITE);   /* 白色中心 */
}

/**
 * @brief  路径轨迹点 (3×3 纯色, 无白心), 用于目标点擦除/轨迹
 */
static void draw_trail_point(int x, int y, uint16_t color)
{
    if (x < 2)       x = 2;
    if (x >= MAP_W - 2)  x = MAP_W - 3;
    if (y < 2)       y = 2;
    if (y >= MAP_H - 2)  y = MAP_H - 3;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            LCD_DrawPoint(x + i, y + j, color);
        }
    }
}

/**
 * @brief  机器人当前位置点 (2×2 纯色 GREEN)
 */
static void draw_pos_point(int x, int y)
{
    if (x < 1)       x = 1;
    if (x >= MAP_W - 1)  x = MAP_W - 2;
    if (y < 1)       y = 1;
    if (y >= MAP_H - 1)  y = MAP_H - 2;

    LCD_DrawPoint(x,     y,     GREEN);
    LCD_DrawPoint(x + 1, y,     GREEN);
    LCD_DrawPoint(x,     y + 1, GREEN);
    LCD_DrawPoint(x + 1, y + 1, GREEN);
}

/**
 * @brief  机器人路径轨迹点 (2×2 BLUE, 离开后覆盖旧位置)
 * @note   每个位姿点仅渲染 2 次: 1st=GREEN, 2nd=BLUE
 */
static void draw_pos_trail(int x, int y)
{
    if (x < 1)       x = 1;
    if (x >= MAP_W - 1)  x = MAP_W - 2;
    if (y < 1)       y = 1;
    if (y >= MAP_H - 1)  y = MAP_H - 2;

    LCD_DrawPoint(x,     y,     BLUE);
    LCD_DrawPoint(x + 1, y,     BLUE);
    LCD_DrawPoint(x,     y + 1, BLUE);
    LCD_DrawPoint(x + 1, y + 1, BLUE);
}

/* ==================================================================
 *  网格绘制 (仅首次, 参考 screen_display_map)
 * ================================================================== */
static void draw_map_grid(void)
{
    int i, x, y;

    /* 竖线: -6m ~ +6m, 间隔 1m */
    for (i = -6; i <= 6; i++) {
        x = standardizing_x((float)i);
        if (x < 0 || x >= MAP_W) continue;
        LCD_DrawLine(x, MAP_Y, x, MAP_Y + MAP_H - 1,
                     (i == 0) ? WHITE : GRAY);
    }

    /* 横线: -2m ~ +5m, 间隔 1m */
    for (i = -2; i <= 5; i++) {
        y = standardizing_y((float)i);
        if (y < 0 || y >= MAP_H) continue;
        LCD_DrawLine(MAP_X, y, MAP_X + MAP_W - 1, y,
                     (i == 0) ? WHITE : GRAY);
    }
}

/* ==================================================================
 *  状态区标签 (仅首次绘制)
 * ================================================================== */
static void draw_status_labels(void)
{
    /* 分隔线 */
    LCD_DrawLine(0, STATUS_Y, LCD_W - 1, STATUS_Y, WHITE);

    /* Row 1: YAW / X / Y */
    LCD_ShowString(2,   ROW1_Y, (const uint8_t *)"YAW:", WHITE, BLACK, 16, 0);
    LCD_ShowString(90,  ROW1_Y, (const uint8_t *)"X:",   GREEN, BLACK, 16, 0);
    LCD_ShowString(170, ROW1_Y, (const uint8_t *)"Y:",   GREEN, BLACK, 16, 0);

    /* Row 2: TX / TY / VIS */
    LCD_ShowString(2,   ROW2_Y, (const uint8_t *)"TX:",  YELLOW, BLACK, 16, 0);
    LCD_ShowString(88,  ROW2_Y, (const uint8_t *)"TY:",  YELLOW, BLACK, 16, 0);
    LCD_ShowString(174, ROW2_Y, (const uint8_t *)"VIS:", WHITE,  BLACK, 16, 0);

    /* Row 3: BAD + mode */
    LCD_ShowString(2, ROW3_Y, (const uint8_t *)"BAD:", WHITE, BLACK, 16, 0);
    LCD_ShowString(210, ROW3_Y,
        screen_vision->cmd == VISION_MODE_COORDINATE ? (const uint8_t *)"POS" : (const uint8_t *)"OFF",
        screen_vision->cmd == VISION_MODE_COORDINATE ? GREEN : YELLOW, BLACK, 16, 0);
}

/* ==================================================================
 *  PUBLIC API
 * ================================================================== */

/**
 * @brief 传入模块数据指针, 由 RobotCMDInit() 调用
 */
void ScreenNavInit(OpticalFlowInstance *flow, attitude_t *ins,
                   Vision_Recv_s *vision, RC_ctrl_t *rc)
{
    screen_flow   = flow;
    screen_ins    = ins;
    screen_vision = vision;
    screen_rc     = rc;
}

/**
 * @brief LCD 导航显示任务 (10Hz, osPriorityLow)
 */
__attribute__((noreturn)) void StartScreenTask(void const *argument)
{
    const OpticalFlow_Data_s *flow_data;
    float pos_x, pos_y, target_x, target_y, yaw, bad_ratio;
    int   sx, sy, tx, ty;
    int   prev_sx = -1, prev_sy = -1;   /* 上一周期屏幕坐标, -1 表示无效 */
    int   prev_tx = -1, prev_ty = -1;   /* 上一周期目标屏幕坐标 */
    float yaw_offset = 0.0f;            /* 中键归零时的 YAW 偏置 */
    uint8_t center_handled = 0;         /* 中键上升沿检测, 防连发 */
    uint8_t vis_online;
    const char *ctrl_str, *chassis_str;
    uint16_t vis_color, bad_color, ctrl_color, chassis_color;

    /* ===== 首次初始化: 清屏 + 网格 + 标签 ===== */
    LCD_Init();
    Key_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    draw_map_grid();
    LCD_Fill(0, STATUS_Y, LCD_W - 1, LCD_H - 1, BLACK);
    draw_status_labels();

    (void)argument;

    for (;;) {
        /* ===== 模块指针有效性检查 ===== */
        if (!screen_flow || !screen_ins || !screen_vision || !screen_rc) {
            osDelay(100);
            continue;
        }

        /* ===== 读取数据 ===== */
        flow_data = OpticalFlowGetData(screen_flow);
        if (!flow_data) {
            osDelay(100);
            continue;
        }

        pos_x     = flow_data->position_x_global;
        pos_y     = flow_data->position_y_global;
        bad_ratio = flow_data->bad_frame_ratio;
        yaw       = screen_ins->Yaw;
        target_x  = screen_vision->target_x;
        target_y  = screen_vision->target_y;
        vis_online = VisionIsOnline();

        /* 控制模式: 左开关上=MANU(红), 下=AUTO(蓝) */
        if (switch_is_up(screen_rc[TEMP].rc.switch_left)) {
            ctrl_str = "MANU"; ctrl_color = RED;
        } else if (switch_is_down(screen_rc[TEMP].rc.switch_left)) {
            ctrl_str = "AUTO"; ctrl_color = BLUE;
        } else {
            ctrl_str = "----"; ctrl_color = WHITE;
        }
        /* 底盘模式: 右开关上=FREE(红), 下=LOCK(蓝) */
        if (switch_is_down(screen_rc[TEMP].rc.switch_right)) {
            chassis_str = "LOCK"; chassis_color = BLUE;
        } else {
            chassis_str = "FREE"; chassis_color = RED;
        }

        /* ===== 中键: 清屏 + 位姿归零 (上升沿触发, 防连发) ===== */
        if (Key_Scan() == KEY_CENTER) {
            if (!center_handled) {
                center_handled = 1;
                /* 清屏重绘 */
                LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
                draw_map_grid();
                LCD_Fill(0, STATUS_Y, LCD_W - 1, LCD_H - 1, BLACK);
                draw_status_labels();
                /* 位姿归零 */
                yaw_offset = yaw;
                OpticalFlowResetPosition(screen_flow);
                pos_x = 0.0f;  pos_y = 0.0f;
                prev_sx = -1;  prev_sy = -1;
                prev_tx = -1;  prev_ty = -1;
            }
        } else {
            center_handled = 0;
        }

        /* 偏航角去偏置, 归一化到 [-180, 180] */
        {
            float dy = yaw - yaw_offset;
            while (dy > 180.0f)  dy -= 360.0f;
            while (dy < -180.0f) dy += 360.0f;
            yaw = dy;
        }

        /* ===== 绘制机器人位姿 (2×2 像素, 两阶段渲染) ===== */
        sx = standardizing_x(pos_x);
        sy = standardizing_y(pos_y);

        if (prev_sx < 0) {
            /* 首次: 直接画 2×2 GREEN */
            draw_pos_point(sx, sy);
        } else if (sx != prev_sx || sy != prev_sy) {
            /* 位置移动: 旧位置→2×2 BLUE 路径色, 新位置→2×2 GREEN */
            draw_pos_trail(prev_sx, prev_sy);
            draw_pos_point(sx, sy);
        }
        /* 位置未变: 跳过 */
        prev_sx = sx;
        prev_sy = sy;

        /* ===== 绘制目标点 (同理: 旧目标→消隐, 新目标→RED) ===== */
        if (target_x != 0.0f || target_y != 0.0f) {
            tx = standardizing_x(target_x);
            ty = standardizing_y(target_y);

            if (prev_tx < 0) {
                draw_big_point(tx, ty, RED);
            } else if (tx != prev_tx || ty != prev_ty) {
                /* 目标移动: 旧目标用背景色 BLACK 擦除, 新目标画 RED */
                draw_trail_point(prev_tx, prev_ty, BLACK);
                draw_big_point(tx, ty, RED);
            }
            prev_tx = tx;
            prev_ty = ty;
        } else {
            /* 目标消失 (target=0,0): 擦除旧目标 */
            if (prev_tx >= 0) {
                draw_trail_point(prev_tx, prev_ty, BLACK);
                prev_tx = -1;
                prev_ty = -1;
            }
        }

        /* ============================================================
         *  刷新状态区数值 (擦旧值 → 写新值, 标签不重绘)
         * ============================================================ */

        /* --- Row 1: YAW, X, Y --- */
        /* YAW: signed, ±XXX.X → 6 chars × 8px = 48px, ends at x=82 */
        LCD_Fill(34, ROW1_Y, 82, ROW1_Y + 15, BLACK);
        LCD_ShowFloatNum(34, ROW1_Y, yaw, 3, 1, WHITE, BLACK, 16);

        /* X: signed, ±XX.XX → 7 chars × 8px = 56px, ends at x=162 */
        LCD_Fill(106, ROW1_Y, 162, ROW1_Y + 15, BLACK);
        LCD_ShowFloatNum(106, ROW1_Y, pos_x, 3, 2, GREEN, BLACK, 16);

        /* Y: signed, ±XX.XX → 7 chars × 8px = 56px, ends at x=242 */
        LCD_Fill(186, ROW1_Y, 242, ROW1_Y + 15, BLACK);
        LCD_ShowFloatNum(186, ROW1_Y, pos_y, 3, 2, GREEN, BLACK, 16);

        /* --- Row 2: TX, TY, VIS --- */
        /* TX: signed, ±XX.XX → ends at x=84 */
        LCD_Fill(28, ROW2_Y, 84, ROW2_Y + 15, BLACK);
        LCD_ShowFloatNum(28, ROW2_Y, target_x, 3, 2, YELLOW, BLACK, 16);

        /* TY: signed, ±XX.XX → ends at x=170 */
        LCD_Fill(114, ROW2_Y, 170, ROW2_Y + 15, BLACK);
        LCD_ShowFloatNum(114, ROW2_Y, target_y, 3, 2, YELLOW, BLACK, 16);

        vis_color = vis_online ? GREEN : RED;
        /* VIS: 1 digit → 8px, ends at x=216 */
        LCD_Fill(208, ROW2_Y, 216, ROW2_Y + 15, BLACK);
        LCD_ShowIntNum(208, ROW2_Y, vis_online, 1, vis_color, BLACK, 16);

        /* --- Row 3: BAD + mode --- */
        if (bad_ratio > 0.50f)
            bad_color = RED;
        else if (bad_ratio > 0.20f)
            bad_color = YELLOW;
        else
            bad_color = GREEN;

        /* BAD: 0~100.0% → ends at x=82 */
        LCD_Fill(42, ROW3_Y, 82, ROW3_Y + 15, BLACK);
        LCD_ShowFloatNum1(42, ROW3_Y, bad_ratio * 100.0f, 3, 1, bad_color, BLACK, 16);

        /* 控制模式 AUTO/MANU → 4 chars × 8px = 32px, ends at x=132 */
        LCD_Fill(100, ROW3_Y, 132, ROW3_Y + 15, BLACK);
        LCD_ShowString(100, ROW3_Y, (const uint8_t *)ctrl_str, ctrl_color, BLACK, 16, 0);

        /* 底盘模式 FREE/LOCK → 4 chars × 8px = 32px, ends at x=187 */
        LCD_Fill(155, ROW3_Y, 187, ROW3_Y + 15, BLACK);
        LCD_ShowString(155, ROW3_Y, (const uint8_t *)chassis_str, chassis_color, BLACK, 16, 0);

        osDelay(100);  /* 10Hz */
    }
}
