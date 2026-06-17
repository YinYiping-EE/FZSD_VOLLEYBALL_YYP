/**
 * @file optical_flow.h
 * @brief 光流模块封装接口.
 * @todo 目前只测量了近距离输出位移变化，远距离由于条件不成熟测出来数据误差较大
 *
 * 当前支持优象 UPIXELS 协议,负责串口接收、协议解析、位移换算和在线检测.
 */
#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "stdint.h"
#include "main.h"
#include "bsp_usart.h"
#include "daemon.h"

/** 支持同时注册的光流模块数量. */
#define OPTICAL_FLOW_MAX_INSTANCE 2
/** UPIXELS 协议固定帧长: 2 字节头 + 10 字节 payload + XOR + 帧尾. */
#define OPTICAL_FLOW_UPIXELS_FRAME_LEN 14
/** UPIXELS 协议有效载荷长度. */
#define OPTICAL_FLOW_UPIXELS_PAYLOAD_LEN 10
/** 默认角位移缩放系数,flow_integral = radians * 10000. */
#define OPTICAL_FLOW_DEFAULT_SCALE 10000.0f

/**
 * @brief 光流模块通信协议类型.
 */
typedef enum
{
    OPTICAL_FLOW_UPIXELS = 0, /**< 优象 UPIXELS 协议. */
} OpticalFlow_Protocol_e;

/**
 * @brief UPIXELS 协议原始 payload.
 *
 * 所有多字节字段均为小端序.
 */
#pragma pack(1)
typedef struct
{
    int16_t flow_x_integral;       /**< X 方向角位移积分值,单位 rad * 10000. */
    int16_t flow_y_integral;       /**< Y 方向角位移积分值,单位 rad * 10000. */
    uint16_t integration_timespan; /**< 本帧积分时间,单位 us. */
    uint16_t ground_distance;      /**< TOF 测距高度,单位 mm,0xFFFF 表示超量程. */
    uint8_t valid;                 /**< 光流置信度,0 无效,255 最可靠. */
    uint8_t tof_confidence;        /**< TOF 测距置信度. */
} OpticalFlow_Upixels_Raw_s;
#pragma pack()

/**
 * @brief 光流模块应用层数据.
 *
 * raw 保存原始协议字段,其余字段为模块换算后的定位数据.
 */
typedef struct
{
    OpticalFlow_Upixels_Raw_s raw; /**< 原始协议数据. */

    float delta_angle_x; /**< X 方向单帧角位移,单位 rad. */
    float delta_angle_y; /**< Y 方向单帧角位移,单位 rad. */
    float delta_x;       /**< X 方向单帧位移,单位 m. */
    float delta_y;       /**< Y 方向单帧位移,单位 m. */
    float position_x;    /**< X 方向累计位移,单位 m. */
    float position_y;    /**< Y 方向累计位移,单位 m. */
    float velocity_x;    /**< X 方向速度,单位 m/s. */
    float velocity_y;    /**< Y 方向速度,单位 m/s. */

    float delta_x_global;    /**< 世界系 X 方向单帧位移,单位 m. */
    float delta_y_global;    /**< 世界系 Y 方向单帧位移,单位 m. */
    float position_x_global; /**< 世界系 X 方向累计位移,单位 m. */
    float position_y_global; /**< 世界系 Y 方向累计位移,单位 m. */
    float velocity_x_global; /**< 世界系 X 方向速度,单位 m/s. */
    float velocity_y_global; /**< 世界系 Y 方向速度,单位 m/s. */

    uint32_t frame_count;  /**< 通过帧校验的通信帧计数. */
    uint32_t update_count; /**< 通过质量门限并更新定位数据的帧计数. */
    uint8_t updated;       /**< 新有效定位数据标志,读取后调用 OpticalFlowClearUpdated() 清除. */
} OpticalFlow_Data_s;

/** 光流模块实例前向声明. */
typedef struct optical_flow_instance OpticalFlowInstance;

/**
 * @brief 光流模块初始化配置.
 */
typedef struct
{
    UART_HandleTypeDef *usart_handle; /**< 光流模块连接的 UART 句柄. */
    OpticalFlow_Protocol_e protocol;  /**< 协议类型,当前使用 OPTICAL_FLOW_UPIXELS. */

    float flow_scale;            /**< 角位移缩放系数,填 0 使用 OPTICAL_FLOW_DEFAULT_SCALE. */
    uint8_t swap_xy;             /**< 安装方向修正: 1 表示交换 X/Y 轴. */
    int8_t x_direction;          /**< 安装方向修正: X 方向符号,填 0 默认 1. */
    int8_t y_direction;          /**< 安装方向修正: Y 方向符号,填 0 默认 1. */
    uint8_t min_valid_threshold; /**< 光流置信度门限,填 0 默认 50. */
    uint8_t enable_global_frame; /**< 使能偏航角全局坐标系变换,非 0 使能,默认 0 关闭. */

    uint16_t daemon_reload_count;                   /**< 离线检测计数,填 0 默认 10. */
    void (*offline_callback)(void *);               /**< 模块离线回调,为空时使用默认串口重启逻辑. */
    void (*update_callback)(OpticalFlowInstance *); /**< 有效定位数据更新后的用户回调. */
} OpticalFlow_Init_Config_s;

/**
 * @brief 光流模块实例.
 *
 * 由 OpticalFlowInit() 创建并返回给应用层保存.
 */
struct optical_flow_instance
{
    USARTInstance *usart_instance;    /**< BSP 串口实例. */
    DaemonInstance *daemon;           /**< 在线检测实例. */
    OpticalFlow_Init_Config_s config; /**< 初始化参数副本. */
    OpticalFlow_Data_s data;          /**< 最新解析数据. */

    float yaw_deg; /**< 当前偏航角(度),由 OpticalFlowSetYaw() 更新,用于世界系旋转. */

    uint8_t payload[OPTICAL_FLOW_UPIXELS_PAYLOAD_LEN]; /**< 正在接收的 payload 缓存. */
    uint8_t payload_idx;                               /**< payload 接收下标. */
    uint8_t rx_state;                                  /**< 协议解析状态机状态. */
    uint8_t xor_calc;                                  /**< 本地计算的 payload 异或校验. */
    uint8_t xor_recv;                                  /**< 帧中接收到的异或校验. */
};

/**
 * @brief 注册 UART 光流模块实例.
 *
 * @param config 初始化配置,部分可选项填 0 时使用默认值.
 * @return OpticalFlowInstance* 光流模块实例指针.
 */
OpticalFlowInstance *OpticalFlowInit(OpticalFlow_Init_Config_s *config);

/**
 * @brief 获取最新解析数据.
 *
 * @param instance 光流模块实例.
 * @return const OpticalFlow_Data_s* 最新数据指针,instance 为空时返回 NULL.
 */
const OpticalFlow_Data_s *OpticalFlowGetData(OpticalFlowInstance *instance);

/**
 * @brief 清除 updated 标志.
 *
 * @param instance 光流模块实例.
 */
void OpticalFlowClearUpdated(OpticalFlowInstance *instance);

/**
 * @brief 将累计位移清零,用于重置光流定位原点.
 *
 * @param instance 光流模块实例.
 */
void OpticalFlowResetPosition(OpticalFlowInstance *instance);

/**
 * @brief 设置累计位移,用于和外部定位或上层坐标系同步.
 *
 * @param instance 光流模块实例.
 * @param x X 方向累计位移,单位 m.
 * @param y Y 方向累计位移,单位 m.
 */
void OpticalFlowSetPosition(OpticalFlowInstance *instance, float x, float y);

/**
 * @brief 更新当前偏航角,用于世界坐标系旋转变换.
 *
 * 由应用层在读取光流数据前调用,传入当前 IMU 偏航角.\n
 * 当 enable_global_frame 使能时,ApplyPayload 会使用此角度将机体系位移旋转到世界系.
 *
 * @param instance 光流模块实例.
 * @param yaw_deg 当前偏航角,单位度.
 */
void OpticalFlowSetYaw(OpticalFlowInstance *instance, float yaw_deg);

/**
 * @brief 判断模块是否在线.
 *
 * @param instance 光流模块实例.
 * @return uint8_t 1: 在线,0: 离线.
 */
uint8_t OpticalFlowIsOnline(OpticalFlowInstance *instance);

#endif
