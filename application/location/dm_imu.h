/**
 * @file dm_imu.h
 * @brief DM-IMU-L1 六轴IMU模块 CAN驱动,仅支持active模式欧拉角解析
 *
 * IMU上电后自动以1kHz向CAN总线推送数据帧。本驱动只解析欧拉角帧(data_type=3),
 * 其余帧(加速度/角速度/四元数)丢弃。
 *
 * @version 1.0
 * @date 2026-06-07
 */

#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stdint.h"
#include "bsp_can.h"   /* 需要 CAN_Init_Config_s 和 FDCAN_HandleTypeDef */

/* 数值范围常量,用于uint16到float的映射 */
#define DM_IMU_ACCEL_MAX     (235.2f)
#define DM_IMU_ACCEL_MIN    (-235.2f)
#define DM_IMU_GYRO_MAX      (34.88f)
#define DM_IMU_GYRO_MIN     (-34.88f)
#define DM_IMU_PITCH_MAX     (90.0f)
#define DM_IMU_PITCH_MIN    (-90.0f)
#define DM_IMU_ROLL_MAX     (180.0f)
#define DM_IMU_ROLL_MIN    (-180.0f)
#define DM_IMU_YAW_MAX      (180.0f)
#define DM_IMU_YAW_MIN     (-180.0f)

/* 寄存器ID */
#define DM_IMU_REG_REBOOT       0
#define DM_IMU_REG_ACCEL_DATA   1
#define DM_IMU_REG_GYRO_DATA    2
#define DM_IMU_REG_EULER_DATA   3
#define DM_IMU_REG_QUAT_DATA    4
#define DM_IMU_REG_SET_ZERO     5
#define DM_IMU_REG_SAVE_PARAM   254
#define DM_IMU_REG_RESTORE      255

/* 指令类型 */
#define DM_IMU_CMD_READ  0
#define DM_IMU_CMD_WRITE 1

/* 默认离线检测计数: 100次无数据视为离线(100ms @ 1kHz) */
#define DM_IMU_DEFAULT_DAEMON_COUNT 100

/**
 * @brief IMU数据结构,包含欧拉角和备用角速度
 */
typedef struct
{
    float pitch;    /**< 俯仰角 (deg) */
    float roll;     /**< 翻滚角 (deg) */
    float yaw;      /**< 偏航角 (deg) */
    float gyro[3];  /**< 角速度 (rad/s), 备用 */
    uint8_t updated; /**< 新数据标志, 读取后调用 DM_IMU_ClearUpdated() 清除 */
} DM_IMU_Data_s;

/**
 * @brief IMU初始化配置
 */
typedef struct
{
    uint32_t tx_id;                     /**< 发往IMU的CAN ID (写寄存器用) */
    uint32_t rx_id;                     /**< IMU推送数据的CAN ID (接收用) */
    FDCAN_HandleTypeDef *can_handle;   /**< CAN总线句柄 */
    uint16_t daemon_count;             /**< 离线检测计数, 默认 DM_IMU_DEFAULT_DAEMON_COUNT */
    void (*offline_callback)(void *);  /**< IMU离线回调 */
} DM_IMU_Init_Config_s;

/**
 * @brief 初始化DM-IMU-L1
 *
 * 注册CAN接收回调、初始化daemon离线检测。
 *
 * @param config 初始化配置指针
 */
void DM_IMU_Init(DM_IMU_Init_Config_s *config);

/**
 * @brief 获取最新IMU数据
 *
 * 若自上次清除后有新数据到达, 会触发欧拉角解析(从raw uint16映射到float)。\n
 * 解析完成后清除内部标志。返回的数据指针指向静态内部数据, 不用释放。
 *
 * @return const DM_IMU_Data_s* 最新数据指针
 */
const DM_IMU_Data_s *DM_IMU_GetData(void);

/**
 * @brief 清除updated标志
 */
void DM_IMU_ClearUpdated(void);

/**
 * @brief 判断IMU是否在线
 *
 * @return uint8_t 1:在线, 0:离线
 */
uint8_t DM_IMU_IsOnline(void);

/**
 * @brief 将当前yaw角归零
 */
void DM_IMU_SetZero(void);

/**
 * @brief 保存当前参数到IMU内部flash
 */
void DM_IMU_SaveParams(void);

/**
 * @brief 重启IMU
 */
void DM_IMU_Reboot(void);

#endif /* __DM_IMU_H */
