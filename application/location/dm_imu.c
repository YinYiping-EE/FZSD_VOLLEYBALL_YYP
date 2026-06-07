/**
 * @file dm_imu.c
 * @brief DM-IMU-L1 六轴IMU模块 CAN驱动实现
 *
 * IMU上电后以active模式1kHz向CAN总线推送欧拉角/加速度/角速度/四元数帧。
 * 本驱动仅处理和解析欧拉角帧(data_type=3),其余帧丢弃。
 *
 * 接收路径: FDCAN RX中断 → CANInstance回调 → DM_IMU_RxCallback(ISR)
 * → 仅拷贝raw数据并设标志 → 应用层调用DM_IMU_GetData()触发解算
 *
 * @version 1.0
 * @date 2026-06-07
 */

#include "dm_imu.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include "bsp_log.h"
#include "daemon.h"

/* ===== 静态全局数据(单例) ===== */
static CANInstance    *imu_can_ins;   /**< bsp_can实例,用于CAN收发 */
static DaemonInstance *imu_daemon;    /**< daemon实例,用于离线检测 */
static DM_IMU_Data_s   imu_data;      /**< 解析后的IMU数据 */
static DM_IMU_Init_Config_s imu_config; /**< 初始化配置副本 */

static uint8_t  rx_raw_buf[8];        /**< ISR中保存的原始8字节数据 */
static uint8_t  rx_new_data;          /**< 新数据标志,ISR设1,GetData后清0 */

/* ===== 内部工具函数 ===== */

/**
 * @brief 将指定位数的无符号整数映射到浮点数范围
 *
 * @param x_int  无符号整数值
 * @param x_min  范围最小值
 * @param x_max  范围最大值
 * @param bits   位宽
 * @return float 映射后的浮点数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

/**
 * @brief 欧拉角帧解析 (data_type=3)
 *
 * 帧格式: [3] [pitch_low] [pitch_high] [yaw_low] [yaw_high] [roll_low] [roll_high] [0]\n
 * 注: 字节序与accel/gyro不同, pitch->yaw->roll顺序。
 *
 * @param data 8字节原始数据
 */
static void parse_euler(const uint8_t *data)
{
    int euler[3];

    euler[0] = (data[3] << 8) | data[2];  /* pitch */
    euler[1] = (data[5] << 8) | data[4];  /* yaw   */
    euler[2] = (data[7] << 8) | data[6];  /* roll  */

    imu_data.pitch = uint_to_float(euler[0], DM_IMU_PITCH_MIN, DM_IMU_PITCH_MAX, 16);
    imu_data.yaw   = uint_to_float(euler[1], DM_IMU_YAW_MIN,   DM_IMU_YAW_MAX,   16);
    imu_data.roll  = uint_to_float(euler[2], DM_IMU_ROLL_MIN,  DM_IMU_ROLL_MAX,  16);
}

/* ===== CAN接收回调 (ISR上下文) ===== */

/**
 * @brief CAN接收中断回调
 *
 * 在FDCAN RX FIFO中断中被调用,仅做最轻量的操作:\n
 * 判断帧类型是否为欧拉角 → 拷贝8字节 → 设标志 → 喂狗。
 *
 * @param can_ins 触发回调的CAN实例指针
 */
static void DM_IMU_RxCallback(CANInstance *can_ins)
{
    /* 仅处理欧拉角帧 (data_type == 3) */
    if (can_ins->rx_buff[0] != 3)
        return;

    memcpy(rx_raw_buf, can_ins->rx_buff, 8);
    rx_new_data = 1;
    DaemonReload(imu_daemon);
}

/* ===== 离线回调 ===== */

/**
 * @brief IMU离线处理
 *
 * 清除updated标志,记录警告日志。
 *
 * @param id 未使用
 */
static void DM_IMU_LostCallback(void *id)
{
    (void)id;
    imu_data.updated = 0;
    LOGWARNING("[dm_imu] IMU offline");
}

/* ===== 发送指令 ===== */

/**
 * @brief 向IMU发送8字节指令帧
 *
 * 帧格式: {0xCC, reg_id, cmd_type, 0xDD, data[4]}
 *
 * @param reg_id   寄存器地址
 * @param cmd_type 命令类型(DM_IMU_CMD_READ/DM_IMU_CMD_WRITE)
 * @param val      写入值(低4字节有效)
 */
static void imu_send_cmd(uint8_t reg_id, uint8_t cmd_type, uint32_t val)
{
    if (imu_can_ins == NULL)
        return;

    imu_can_ins->tx_buff[0] = 0xCC;
    imu_can_ins->tx_buff[1] = reg_id;
    imu_can_ins->tx_buff[2] = cmd_type;
    imu_can_ins->tx_buff[3] = 0xDD;
    memcpy(&imu_can_ins->tx_buff[4], &val, 4);

    CANTransmit(imu_can_ins, 1);  /* 1ms超时 */
}

/* ===== 公开API ===== */

/**
 * @brief 初始化DM-IMU-L1
 */
void DM_IMU_Init(DM_IMU_Init_Config_s *config)
{
    memcpy(&imu_config, config, sizeof(DM_IMU_Init_Config_s));

    /* 注册CAN实例 */
    CAN_Init_Config_s can_config = {
        .can_handle            = config->can_handle,
        .tx_id                 = config->tx_id,
        .rx_id                 = config->rx_id,
        .can_module_callback   = DM_IMU_RxCallback,
        .id                    = NULL,
    };
    imu_can_ins = CANRegister(&can_config);

    /* 注册daemon */
    Daemon_Init_Config_s daemon_config = {
        .owner_id     = NULL,
        .reload_count = config->daemon_count ? config->daemon_count : DM_IMU_DEFAULT_DAEMON_COUNT,
        .callback     = config->offline_callback ? config->offline_callback : DM_IMU_LostCallback,
    };
    imu_daemon = DaemonRegister(&daemon_config);

    LOGINFO("[dm_imu] IMU init success, tx_id=0x%x, rx_id=0x%x",
            (unsigned int)config->tx_id, (unsigned int)config->rx_id);
}

/**
 * @brief 获取最新IMU数据
 */
const DM_IMU_Data_s *DM_IMU_GetData(void)
{
    if (rx_new_data)
    {
        parse_euler(rx_raw_buf);
        imu_data.updated = 1;
        rx_new_data = 0;
    }
    return &imu_data;
}

/**
 * @brief 清除updated标志
 */
void DM_IMU_ClearUpdated(void)
{
    imu_data.updated = 0;
}

/**
 * @brief 判断IMU是否在线
 */
uint8_t DM_IMU_IsOnline(void)
{
    return imu_daemon ? DaemonIsOnline(imu_daemon) : 0;
}

/**
 * @brief 将当前yaw角归零
 */
void DM_IMU_SetZero(void)
{
    imu_send_cmd(DM_IMU_REG_SET_ZERO, DM_IMU_CMD_WRITE, 0);
}

/**
 * @brief 保存当前参数到IMU内部flash
 */
void DM_IMU_SaveParams(void)
{
    imu_send_cmd(DM_IMU_REG_SAVE_PARAM, DM_IMU_CMD_WRITE, 0);
}

/**
 * @brief 重启IMU
 */
void DM_IMU_Reboot(void)
{
    imu_send_cmd(DM_IMU_REG_REBOOT, DM_IMU_CMD_WRITE, 0);
}
