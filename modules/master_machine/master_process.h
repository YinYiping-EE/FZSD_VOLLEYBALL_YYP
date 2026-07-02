#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"

/* ---------------- 帧长度（与协议一致） ---------------- */
#define VISION_RECV_SIZE 22u   // planArray 长度
#define VISION_SEND_SIZE 17u   // robotArray 长度

/* ---------------- 新协议枚举 ---------------- */
typedef enum {
    MODE_IDLE   = 0,
    MODE_REMOTE = 1,
    MODE_SELF   = 2
} Robot_Mode_e;

typedef enum {
    STATE_WAITING_PLAN  = 0,
    STATE_RECEIVED_PLAN = 1,
    STATE_CATCHING      = 2,
    STATE_OVER          = 3
} Robot_State_e;

typedef enum {
    CMD_MOVE_PLAN = 0,
    CMD_OFFSET    = 1   /**< 像素误差 PID 跟踪模式 */
} Plan_Cmd_e;

/* ---------------- 数据结构体（名称不变，字段重新定义） ---------------- */
#pragma pack(1)

/**
 * @brief 接收数据结构体（planArray）
 *        上位机下发的接球规划目标
 */
typedef struct {
    uint8_t cmd;              // CMD_MOVE_PLAN=0 坐标导航, CMD_OFFSET=1 误差跟踪
    float target_x;           // 接球目标位置 X (m)
    float target_y;           // 接球目标位置 Y (m)
    float target_yaw;         // 目标机器人朝向 (度)
    float target_time;        // 球到击球点的预测时间 (s)
} Vision_Recv_s;

/**
 * @brief 发送数据结构体（robotArray）
 *        下位机上报的当前机器人状态
 */
typedef struct {
    Robot_Mode_e  mode;       // 当前执行模式
    Robot_State_e state;      // 接球阶段状态
    float robot_x;            // 机器人当前 X 坐标 (m)
    float robot_y;            // 机器人当前 Y 坐标 (m)
    float robot_yaw;          // 机器人当前航向角 (度)
} Vision_Send_s;

#pragma pack()

/* ---------------- 外部接口（与原来完全一致） ---------------- */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
void VisionSend(Vision_Send_s *send);
uint8_t VisionIsOnline(void);

#endif // MASTER_PROCESS_H