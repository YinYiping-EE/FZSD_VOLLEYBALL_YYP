/**
 * @file remote_control.c
 * @brief  遥控器模块,富斯i6x遥控器数据解析模块,适配SBUS协议,美国手映射,无拨轮,SWB/SWC开关适配
 *
 */
#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

//#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

#define REMOTE_CONTROL_FRAME_SIZE 25u // SBUS帧大小25字节（富斯IA6B）

//#define SBUS_STICK_RANGE 1024   // ch[i] = raw - offset 后的理论幅值
#define SBUS_STICK_RANGE 784   // ch[i] = raw - offset 后的实测值
#define RC_STICK_RANGE   660    // 原框架DT7遥控器摇杆幅值   
// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > RC_STICK_RANGE)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

// /**
//  * @brief 遥控器数据解析
//  *
//  * @param sbus_buf 接收buffer
//  */
// static void sbus_to_rc(const uint8_t *sbus_buf)
// {
//     // 摇杆,直接解算时减去偏置
//     rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
//     rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
//     rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
//     rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
//     rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
//     RectifyRCjoystick();
//     // 开关,0左1右
//     rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
//     rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

//     // 鼠标解析
//     rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
//     rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
//     rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
//     rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

//     //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
//     *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
//     if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
//         rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
//     else
//         memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
//     if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
//         rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
//     else
//         memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

//     uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
//         key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
//         key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
//         key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
//         key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
//         key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

//     for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
//     {
//         if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
//             continue;
//         // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
//         if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
//         // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
//         if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
//         // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
//         if ((key_with_shift & j) && !(key_last_with_shift & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
//     }

//     memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
// }

/**
 * @brief 富斯i6x遥控器数据sbus解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // ========== 1. 校验SBUS起始字节（可选，增强鲁棒性） ==========
    // if (sbus_buf[0] != 0x0F) { // SBUS起始字节为0x0F
    //     memset(&rc_ctrl[TEMP], 0, sizeof(RC_ctrl_t));
    //     memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
    //     return;
    // }

    // ========== 2. SBUS通道解析（富斯i6x美国手映射） ==========
    // 通道映射（富斯i6x美国手）：
    // Ch1 → 右摇杆水平（rocker_r_） | Ch2 → 右摇杆竖直（rocker_r1）
    // Ch3 → 左摇杆竖直（rocker_l1） | Ch4 → 左摇杆水平（rocker_l_）
    // Ch5 → 左开关（SA）| Ch6 → 右开关（SB）| 拨轮无（置0）
    int16_t ch[6] = {0};
    // SBUS 11位通道解析（前6通道）
    ch[0] = ((sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch1-右水平
    ch[1] = (((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch2-右竖直
    ch[2] = (((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch3-左竖直
    ch[3] = (((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch4-左水平
    ch[4] = (((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch5-左开关SWB
    ch[5] = (((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07FF) - RC_CH_VALUE_OFFSET; // Ch6-右开关SWC

    // ========== 3. 将SBUS中-1024~+1024映射回原框架中的-660~+660 ==========

    for (uint8_t i = 0; i < 4; i++)
    {
        //这里把限幅关了是因为实测发现使用过程中某些通道会出现尖峰（原因不明）
        //尖峰值往往大于实测最大幅值，如果直接限幅到实测最大幅值会导致尖峰值被剪切成正常值
        //因此采用大于实测最大幅值直接裁剪的方式，见RectifyRCjoystick(); 
        // 1) 输入限幅到 [-1024, +1024]（防止接收机输出略超范围）
        // if (ch[i] >  SBUS_STICK_RANGE) ch[i] =  SBUS_STICK_RANGE;
        // if (ch[i] < -SBUS_STICK_RANGE) ch[i] = -SBUS_STICK_RANGE;

        // 2) 线性缩放：ch[i] * 660 / 1024，并做四舍五入
        int32_t tmp = (int32_t)ch[i] * RC_STICK_RANGE;

        // 四舍五入：正数 +512，负数 -512
        if (tmp >= 0) tmp += (SBUS_STICK_RANGE / 2);
        else          tmp -= (SBUS_STICK_RANGE / 2);

        tmp /= SBUS_STICK_RANGE;

        // 3) 输出再限幅到 [-660, +660]
        // if (tmp >  RC_STICK_RANGE) tmp =  RC_STICK_RANGE;
        // if (tmp < -RC_STICK_RANGE) tmp = -RC_STICK_RANGE;

        ch[i] = (int16_t)tmp;
    }
    // 赋值到rc_ctrl
    rc_ctrl[TEMP].rc.rocker_r_ = ch[0];    // 右水平
    rc_ctrl[TEMP].rc.rocker_r1 = ch[1];    // 右竖直
    rc_ctrl[TEMP].rc.rocker_l1 = ch[2];    // 左竖直
    rc_ctrl[TEMP].rc.rocker_l_ = ch[3];    // 左水平
    rc_ctrl[TEMP].rc.dial = 0;             // 富斯i6x无拨轮，置0

    // ========== 4. 开关解析（适配富斯i6x的SWB/SWC开关） ==========
    // 开关数值映射（SBUS通道值：784=下，0=中，-784=上 → 注意之前减去了1024的偏置 ）

    #define SW_THRESHOLD 500 // 阈值，防止抖动

    // 左开关（SWB）→ switch_left
    if (ch[4] >  SW_THRESHOLD) {
        rc_ctrl[TEMP].rc.switch_left = RC_SW_DOWN;    // 下
    } else if (ch[4] <  -SW_THRESHOLD) {
        rc_ctrl[TEMP].rc.switch_left = RC_SW_UP;  // 上
    }  else {
        rc_ctrl[TEMP].rc.switch_left = RC_SW_MID;  // 中（仅三档开关有效）
    }
    // 右开关（SWC）→ switch_right（注意i6x只有SWC开关是3档的）
    if (ch[5] >  SW_THRESHOLD) {
        rc_ctrl[TEMP].rc.switch_right = RC_SW_DOWN;   // 下
    } else if (ch[5] <  -SW_THRESHOLD) {
        rc_ctrl[TEMP].rc.switch_right = RC_SW_UP; // 上
    } else {
        rc_ctrl[TEMP].rc.switch_right = RC_SW_MID;  // 中（仅三档开关有效）
    }

    // ========== 5. 鼠标/键盘置0（SBUS无该数据） ==========
    memset(&rc_ctrl[TEMP].mouse, 0, sizeof(rc_ctrl[TEMP].mouse)); // 鼠标置0
    memset(&rc_ctrl[TEMP].key, 0, sizeof(rc_ctrl[TEMP].key));     // 键盘置0
    memset(&rc_ctrl[TEMP].key_count, 0, sizeof(rc_ctrl[TEMP].key_count)); // 按键计数置0

    // ========== 5. 摇杆矫正（超过±660置0） ==========
    RectifyRCjoystick();    

    // ========== 6. 保存上一次数据（保留原逻辑） ==========
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
}


/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    uint8_t head = rc_usart_instance->recv_buff[0];
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}