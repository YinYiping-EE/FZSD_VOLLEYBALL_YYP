/**
 * @file lcd_test_task.c
 * @brief LCD+Key 测试任务 (10Hz)
 *
 * 初始化 LCD 和四向按键, 循环扫描并在屏幕上显示当前方向 + ADC 原始值。
 * 优先级 osPriorityLow, 不干扰 INS/Motor 等高实时任务。
 */

#include "lcd_test_task.h"
#include "cmsis_os.h"
#include "lcd.h"
#include "key.h"

static const char *dir_names[] = {
    "NONE ",
    "UP   ",
    "DOWN ",
    "LEFT ",
    "RIGHT",
    "CENT "
};

__attribute__((noreturn)) void StartLCDTEST(void const *argument)
{
    KeyDirection dir, last_dir = KEY_NONE;
    uint16_t adc_val;

    /* ---- 初始化 ---- */
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    Key_Init();

    /* ---- 标题 ---- */
    LCD_ShowString(10, 10, (const uint8_t *)"Key Test", WHITE, BLACK, 24, 0);
    LCD_DrawLine(10, 38, LCD_W - 10, 38, GRAY);

    /* ---- 标签 ---- */
    LCD_ShowString(10, 50, (const uint8_t *)"Dir:", YELLOW, BLACK, 16, 0);
    LCD_ShowString(10, 80, (const uint8_t *)"ADC:", YELLOW, BLACK, 16, 0);

    for (;;) {
        dir = Key_Scan();
        adc_val = Key_GetADC();

        /* 方向变化时刷新方向文字 (减少 SPI 传输) */
        if (dir != last_dir) {
            LCD_Fill(60, 50, 180, 66, BLACK);
            LCD_ShowString(60, 50, (const uint8_t *)dir_names[dir], GREEN, BLACK, 16, 0);
            last_dir = dir;
        }

        /* ADC 值每轮刷新 (用于校准阈值) */
        LCD_Fill(60, 80, 180, 96, BLACK);
        LCD_ShowIntNum(60, 80, adc_val, 5, CYAN, BLACK, 16);

        osDelay(100);  /* 10Hz */
    }
}
