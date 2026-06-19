/**
 * @file key.c
 * @brief 四向按键驱动实现
 *
 * PA5  (ADC1_CH19): 4 方向电阻分压 → ADC DMA 循环采样
 * PA15 (GPIO Input): 中键数字输入 → 消抖状态机
 *
 * 移植自 CtrBoard-H7_ALL 工程 Bsp/bsp_user_key.c, 扩展 ADC 方向检测。
 */

#include "key.h"

/* ===== ADC DMA 缓冲区 =====
 * adc_buf[0]: PC4  ADC_CH4  (vbus 电压监测, 由其他模块使用)
 * adc_buf[1]: PA5  ADC_CH19 (四向按键分压输入)
 */
static volatile uint16_t adc_buf[2];

/* ===== PA15 中键消抖状态机 ===== */
typedef enum
{
    S_NO_DETECT,
    S_DETECTING,
    S_DETECTED
} KeyDebounceState;

static KeyDebounceState key_state = S_NO_DETECT;
static uint32_t         key_tick  = 0;

/* ===== 四向按键消抖 ===== */
static KeyDirection last_direction = KEY_NONE;
static uint32_t     dir_tick       = 0;

/* ==================================================================
 *  PUBLIC API
 * ================================================================== */

/**
 * @brief  初始化按键模块
 * @note   启动 ADC1 DMA 循环转换, PA5 数据进入 adc_buf[1]
 */
void Key_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 2);
}

/**
 * @brief  扫描按键方向 (含消抖)
 * @retval 当前按下的方向 (5 向全通过 PA5 ADC 检测)
 */
KeyDirection Key_Scan(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t adc_val;
    KeyDirection dir = KEY_NONE;

    adc_val = (uint16_t)adc_buf[1];

    if (adc_val <= KEY_ADC_CENTER_MAX) {
        dir = KEY_CENTER;
    } else if (adc_val >= KEY_ADC_DOWN_MIN && adc_val <= KEY_ADC_DOWN_MAX) {
        dir = KEY_DOWN;
    } else if (adc_val >= KEY_ADC_UP_MIN && adc_val <= KEY_ADC_UP_MAX) {
        dir = KEY_UP;
    } else if (adc_val >= KEY_ADC_LEFT_MIN && adc_val <= KEY_ADC_LEFT_MAX) {
        dir = KEY_LEFT;
    } else if (adc_val >= KEY_ADC_RIGHT_MIN && adc_val <= KEY_ADC_RIGHT_MAX) {
        dir = KEY_RIGHT;
    }

    /* 消抖: 方向变化需要保持 KEY_DEBOUNCE_MS */
    if (dir != last_direction) {
        last_direction = dir;
        dir_tick = now + KEY_DEBOUNCE_MS;
        return KEY_NONE;
    }

    if (now < dir_tick) {
        return KEY_NONE;
    }

    return dir;
}

/**
 * @brief  原始中键检测 (仅 PA15, 兼容旧接口)
 * @retval 1 = 按下, 0 = 未按下
 */
int8_t Key_Detect(void)
{
    uint32_t now = HAL_GetTick();
    GPIO_PinState pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    if (key_state == S_NO_DETECT) {
        if (pin == GPIO_PIN_RESET) {
            key_state = S_DETECTING;
            key_tick  = now + KEY_DEBOUNCE_MS;
        }
        return 0;
    } else if (key_state == S_DETECTING) {
        if (now >= key_tick) {
            if (pin == GPIO_PIN_RESET) {
                key_state = S_DETECTED;
                return 1;
            } else {
                key_state = S_NO_DETECT;
                return 0;
            }
        }
        return 0;
    } else {  /* S_DETECTED */
        if (pin == GPIO_PIN_SET) {
            key_state = S_NO_DETECT;
        }
        return 0;
    }
}

/**
 * @brief  获取 PA5 ADC 原始值 (用于调试/校准阈值)
 * @retval 16-bit ADC 值 (0~65535)
 */
uint16_t Key_GetADC(void)
{
    return (uint16_t)adc_buf[1];
}
