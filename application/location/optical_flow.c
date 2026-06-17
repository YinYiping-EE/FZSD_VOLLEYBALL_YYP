/**
 * @file optical_flow.c
 * @brief 光流模块封装实现.
 * @todo 目前只测量了近距离输出位移变化，远距离由于条件不成熟测出来数据误差较大
 *
 * 实现 UPIXELS 协议解析、位移换算、串口回调分发和离线检测.
 */
#include "optical_flow.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"
#include "user_lib.h"
#include "general_def.h"

/** 保存所有光流实例,用于串口回调中分发接收到的数据. */
static OpticalFlowInstance *optical_flow_instances[OPTICAL_FLOW_MAX_INSTANCE] = {NULL};
/** 当前已注册的光流实例数量. */
static uint8_t optical_flow_idx;

/**
 * @brief 按小端序读取 int16_t 字段.
 *
 * @param buf 字节缓冲区首地址.
 * @return int16_t 读取结果.
 */
static int16_t OpticalFlowReadI16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/**
 * @brief 按小端序读取 uint16_t 字段.
 *
 * @param buf 字节缓冲区首地址.
 * @return uint16_t 读取结果.
 */
static uint16_t OpticalFlowReadU16(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/**
 * @brief 重置协议解析状态机.
 *
 * @param instance 光流模块实例.
 */
static void OpticalFlowResetParser(OpticalFlowInstance *instance)
{
    instance->rx_state = 0;
    instance->payload_idx = 0;
    instance->xor_calc = 0;
    instance->xor_recv = 0;
}

/**
 * @brief 默认离线处理回调.
 *
 * 清除更新标志并重启 USART DMA 接收.
 *
 * @param id 光流模块实例指针.
 */
static void OpticalFlowDefaultLostCallback(void *id)
{
    OpticalFlowInstance *instance = (OpticalFlowInstance *)id;
    if (instance == NULL)
        return;

    instance->data.updated = 0;
    LOGWARNING("[optical_flow] optical flow lost");
    USARTServiceInit(instance->usart_instance);
}

/**
 * @brief 解析已校验的 payload 并更新光流数据.
 *
 * @param instance 光流模块实例.
 */
static void OpticalFlowApplyPayload(OpticalFlowInstance *instance)
{
    OpticalFlow_Upixels_Raw_s *raw = &instance->data.raw;
    float scale = instance->config.flow_scale;
    float angle_x;
    float angle_y;
    float dx;
    float dy;

    /* 解析 UPIXELS payload 原始字段. */
    raw->flow_x_integral = OpticalFlowReadI16(&instance->payload[0]);
    raw->flow_y_integral = OpticalFlowReadI16(&instance->payload[2]);
    raw->integration_timespan = OpticalFlowReadU16(&instance->payload[4]);
    raw->ground_distance = OpticalFlowReadU16(&instance->payload[6]);
    raw->valid = instance->payload[8];
    raw->tof_confidence = instance->payload[9];

    if (scale == 0.0f)
        scale = OPTICAL_FLOW_DEFAULT_SCALE;

    angle_x = (float)raw->flow_x_integral / scale;
    angle_y = (float)raw->flow_y_integral / scale;
    instance->data.delta_angle_x = angle_x;
    instance->data.delta_angle_y = angle_y;

    /*
     * 能走到这里说明帧头、长度、XOR 和帧尾已经通过校验。
     * daemon 用于判断通信链路是否存活,因此应在质量门限判断前喂狗。
     */
    instance->data.frame_count++;

    if (instance->daemon)
        DaemonReload(instance->daemon);

    /* 低质量光流或 TOF 超量程时,保留上一帧有效定位结果. */
    if (raw->valid < instance->config.min_valid_threshold || raw->ground_distance == 0xFFFF)
        return;

    /* 角位移(rad) * 高度(mm) / 1000 = 实际平移(m). */
    dx = angle_x * (float)raw->ground_distance * 0.001f;
    dy = angle_y * (float)raw->ground_distance * 0.001f;

    /* 根据车底实际安装方向修正光流坐标系到车体坐标系. */
    if (instance->config.swap_xy)
    {
        float tmp = dx;
        dx = dy;
        dy = tmp;
    }

    dx *= (float)instance->config.x_direction;
    dy *= (float)instance->config.y_direction;

    instance->data.delta_x = dx;
    instance->data.delta_y = dy;
    instance->data.position_x += dx;
    instance->data.position_y += dy;

    /* integration_timespan 单位为 us,换算成秒后计算速度. */
    float dt_s = (float)raw->integration_timespan * 0.000001f;
    if (dt_s > 0.0f)
    {
        instance->data.velocity_x = dx / dt_s;
        instance->data.velocity_y = dy / dt_s;
    }

    /*
     * 全局坐标系映射: 用偏航角将机体系位移旋转到世界系.
     * 右手系 +x 前 +y 右, 偏航 CCW 为正.
     * dx_global = dx*cos(θ) - dy*sin(θ)
     * dy_global = dx*sin(θ) + dy*cos(θ)
     */
    if (instance->config.enable_global_frame)
    {
        float yaw_rad = instance->yaw_deg * DEGREE_2_RAD;
        float cy = mcos(yaw_rad);
        float sy = msin(yaw_rad);
        float dgx = dx * cy - dy * sy;
        float dgy = dx * sy + dy * cy;

        instance->data.delta_x_global = dgx;
        instance->data.delta_y_global = dgy;
        instance->data.position_x_global += dgx;
        instance->data.position_y_global += dgy;

        if (dt_s > 0.0f)
        {
            instance->data.velocity_x_global = dgx / dt_s;
            instance->data.velocity_y_global = dgy / dt_s;
        }
    }

    instance->data.updated = 1;
    instance->data.update_count++;

    if (instance->config.update_callback)
        instance->config.update_callback(instance);
}

/**
 * @brief UPIXELS 逐字节协议解析状态机.
 *
 * @param instance 光流模块实例.
 * @param ch 当前接收字节.
 */
static void OpticalFlowParseByte(OpticalFlowInstance *instance, uint8_t ch)
{
    switch (instance->rx_state)
    {
    case 0:
        /* 等待帧头 0xFE. */
        if (ch == 0xFE)
            instance->rx_state = 1;
        break;
    case 1:
        /* UPIXELS payload 长度固定为 0x0A. */
        if (ch == OPTICAL_FLOW_UPIXELS_PAYLOAD_LEN)
        {
            instance->payload_idx = 0;
            instance->xor_calc = 0;
            instance->rx_state = 2;
        }
        else
        {
            OpticalFlowResetParser(instance);
        }
        break;
    case 2:
        /* 接收 10 字节 payload,同时计算异或校验. */
        instance->payload[instance->payload_idx++] = ch;
        instance->xor_calc ^= ch;
        if (instance->payload_idx >= OPTICAL_FLOW_UPIXELS_PAYLOAD_LEN)
            instance->rx_state = 3;
        break;
    case 3:
        /* 保存帧内校验值. */
        instance->xor_recv = ch;
        instance->rx_state = 4;
        break;
    case 4:
        /* 校验帧尾和 XOR,通过后才更新实例数据. */
        if (ch == 0x55 && instance->xor_recv == instance->xor_calc)
            OpticalFlowApplyPayload(instance);
        OpticalFlowResetParser(instance);
        break;
    default:
        OpticalFlowResetParser(instance);
        break;
    }
}

/**
 * @brief BSP USART 接收回调.
 *
 * 将 DMA 缓冲区中的字节交给各实例的协议状态机解析.
 */
static void OpticalFlowRxCallback()
{
    uint16_t recv_len;

    for (uint8_t i = 0; i < optical_flow_idx; i++)
    {
        OpticalFlowInstance *instance = optical_flow_instances[i];
        if (instance == NULL || instance->usart_instance == NULL)
            continue;

        recv_len = instance->usart_instance->recv_buff_size;

        for (uint16_t j = 0; j < recv_len; j++)
            OpticalFlowParseByte(instance, instance->usart_instance->recv_buff[j]);
    }
}

/**
 * @brief 初始化光流模块.
 *
 * 注册 USART、注册 daemon,并保存实例到回调分发表.
 *
 * @param config 初始化配置.
 * @return OpticalFlowInstance* 光流模块实例指针.
 */
OpticalFlowInstance *OpticalFlowInit(OpticalFlow_Init_Config_s *config)
{
    if (config == NULL || config->usart_handle == NULL)
        while (1)
            LOGERROR("[optical_flow] invalid init config");

    if (config->protocol != OPTICAL_FLOW_UPIXELS)
        while (1)
            LOGERROR("[optical_flow] unsupported protocol");

    if (optical_flow_idx >= OPTICAL_FLOW_MAX_INSTANCE)
        while (1)
            LOGERROR("[optical_flow] optical flow instance exceeded max num");

    OpticalFlowInstance *instance = (OpticalFlowInstance *)malloc(sizeof(OpticalFlowInstance));
    if (instance == NULL)
        while (1)
            LOGERROR("[optical_flow] malloc failed");

    memset(instance, 0, sizeof(OpticalFlowInstance));
    memcpy(&instance->config, config, sizeof(OpticalFlow_Init_Config_s));

    /* 对可选配置项做默认值归一化,后续计算路径可直接使用. */
    if (instance->config.flow_scale == 0.0f)
        instance->config.flow_scale = OPTICAL_FLOW_DEFAULT_SCALE;
    if (instance->config.x_direction > 0)
        instance->config.x_direction = 1;
    else if (instance->config.x_direction < 0)
        instance->config.x_direction = -1;
    else
        instance->config.x_direction = 1;

    if (instance->config.y_direction > 0)
        instance->config.y_direction = 1;
    else if (instance->config.y_direction < 0)
        instance->config.y_direction = -1;
    else
        instance->config.y_direction = 1;

    if (instance->config.min_valid_threshold == 0)
        instance->config.min_valid_threshold = 50;

    if (instance->config.enable_global_frame > 1)
        instance->config.enable_global_frame = 1;

    USART_Init_Config_s usart_config = {
        .recv_buff_size = OPTICAL_FLOW_UPIXELS_FRAME_LEN,
        .usart_handle = config->usart_handle,
        .module_callback = OpticalFlowRxCallback,
    };
    instance->usart_instance = USARTRegister(&usart_config);

    /* daemon 只反映通信链路是否持续收到合法帧,不反映该帧是否适合用于定位. */
    Daemon_Init_Config_s daemon_config = {
        .reload_count = config->daemon_reload_count == 0 ? 10 : config->daemon_reload_count,
        .callback = config->offline_callback ? config->offline_callback : OpticalFlowDefaultLostCallback,
        .owner_id = instance,
    };
    instance->daemon = DaemonRegister(&daemon_config);

    optical_flow_instances[optical_flow_idx++] = instance;
    return instance;
}

/**
 * @brief 获取最新解析数据.
 *
 * @param instance 光流模块实例.
 * @return const OpticalFlow_Data_s* 最新数据指针,instance 为空时返回 NULL.
 */
const OpticalFlow_Data_s *OpticalFlowGetData(OpticalFlowInstance *instance)
{
    if (instance == NULL)
        return NULL;

    return &instance->data;
}

/**
 * @brief 清除 updated 标志.
 *
 * @param instance 光流模块实例.
 */
void OpticalFlowClearUpdated(OpticalFlowInstance *instance)
{
    if (instance == NULL)
        return;

    instance->data.updated = 0;
}

/**
 * @brief 清零累计位移.
 *
 * @param instance 光流模块实例.
 */
void OpticalFlowResetPosition(OpticalFlowInstance *instance)
{
    if (instance == NULL)
        return;

    instance->data.position_x = 0.0f;
    instance->data.position_y = 0.0f;
    instance->data.position_x_global = 0.0f;
    instance->data.position_y_global = 0.0f;
}

/**
 * @brief 设置累计位移.
 *
 * @param instance 光流模块实例.
 * @param x X 方向累计位移,单位 m.
 * @param y Y 方向累计位移,单位 m.
 */
void OpticalFlowSetPosition(OpticalFlowInstance *instance, float x, float y)
{
    if (instance == NULL)
        return;

    instance->data.position_x = x;
    instance->data.position_y = y;
}

void OpticalFlowSetYaw(OpticalFlowInstance *instance, float yaw_deg)
{
    if (instance == NULL)
        return;

    instance->yaw_deg = yaw_deg;
}

/**
 * @brief 判断光流模块是否在线.
 *
 * @param instance 光流模块实例.
 * @return uint8_t 1: 在线,0: 离线.
 */
uint8_t OpticalFlowIsOnline(OpticalFlowInstance *instance)
{
    if (instance == NULL || instance->daemon == NULL)
        return 0;

    return DaemonIsOnline(instance->daemon);
}
