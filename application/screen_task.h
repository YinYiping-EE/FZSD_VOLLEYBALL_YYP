/**
 * @file screen_task.h
 * @brief LCD 导航路径+状态单页面显示
 *
 * 上半 280×176 地图 (网格一次性绘制, 位姿点累积不清屏)
 * 下半 280×64  状态参数实时刷新
 *
 * 参考 E03_uart_receiver_demo screen.c 路径显示思路
 */
#ifndef SCREEN_TASK_H
#define SCREEN_TASK_H

#include "optical_flow.h"
#include "ins_task.h"
#include "master_process.h"
#include "remote_control.h"

/**
 * @brief 传入模块数据指针 (在 RobotCMDInit 末尾调用, 所有模块 init 之后)
 */
void ScreenNavInit(OpticalFlowInstance *flow, attitude_t *ins,
                   Vision_Recv_s *vision, RC_ctrl_t *rc);

/**
 * @brief LCD 导航显示 FreeRTOS 任务入口 (10Hz, osPriorityLow)
 */
void StartScreenTask(void const *argument);

#endif /* SCREEN_TASK_H */
