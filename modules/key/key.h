/**
 * @file key.h
 * @brief 四向按键驱动头文件 (PA5 ADC 电阻分压, 5 向)
 *
 * 物理按键是一个 5 向导航键 (上/下/左/右/中), 全部通过 PA5 ADC1_CH19 检测。
 * PA15 也同时拉低, 作为辅助数字检测保留在 Key_Detect() 中。
 *
 * 实测 ADC 值 (16-bit, 3.3V):
 *   中键:    50    上: 25000    下: 13000    左: 38000    右: 51000    空: 64000
 */

#ifndef __KEY_H
#define __KEY_H

#include "gpio.h"
#include "adc.h"

/* ===== 按键方向枚举 ===== */
typedef enum
{
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_CENTER
} KeyDirection;

/* ===== ADC 阈值 (基于实测值, 留死区) ===== */
#define KEY_ADC_CENTER_MIN      0
#define KEY_ADC_CENTER_MAX   3000

#define KEY_ADC_DOWN_MIN     3001
#define KEY_ADC_DOWN_MAX    19000

#define KEY_ADC_UP_MIN      19001
#define KEY_ADC_UP_MAX     31000

#define KEY_ADC_LEFT_MIN    31001
#define KEY_ADC_LEFT_MAX   44000

#define KEY_ADC_RIGHT_MIN   44001
#define KEY_ADC_RIGHT_MAX  57000

#define KEY_DEBOUNCE_MS     100     /* 消抖时间窗 (ms) */

/* ===== API 函数声明 ===== */

/**
 * @brief  初始化按键模块 (启动 ADC1 DMA 循环采样)
 * @note   调用前需确保 MX_ADC1_Init() 已完成。
 *         ADC1 DMA 循环写入 adc_buf[2]: ch4=PC4(vbus), ch19=PA5(key)
 */
void Key_Init(void);

/**
 * @brief  扫描按键方向 (消抖状态机, 建议 10ms 周期调用)
 * @retval KeyDirection - 当前按下的方向
 *         KEY_NONE   - 无按键
 *         KEY_UP     - 上
 *         KEY_DOWN   - 下
 *         KEY_LEFT   - 左
 *         KEY_RIGHT  - 右
 *         KEY_CENTER - 中键按下
 */
KeyDirection Key_Scan(void);

/**
 * @brief  原始按键检测 (仅 PA15 中键, 保留兼容)
 * @retval 1 - 中键刚被按下
 * @retval 0 - 未按下或消抖中
 */
int8_t Key_Detect(void);

/**
 * @brief  获取 PA5 ADC 原始值 (用于调试/校准)
 * @retval 16-bit ADC 原始值 (0~65535)
 */
uint16_t Key_GetADC(void);

#endif /* __KEY_H */
