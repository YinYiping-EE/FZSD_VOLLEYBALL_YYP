/**
 * @file master_process.c
 * @brief  视觉通信模块（适配排球机器人新协议，交换收发方向）
 * @note   下位机接收 planArray → Vision_Recv_s
 *         下位机发送 robotArray → Vision_Send_s
 *         校验：异或从索引1到倒数第2字节（不含包头、校验位、包尾）
 */

#include "master_process.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static USARTInstance *vision_usart_instance;
static Vision_Recv_s recv_data;         // 存储接收到的规划目标
static DaemonInstance *vision_daemon_instance;

/* ========================= 离线回调 ========================= */
static void VisionOfflineCallback(void *id)
{
    (void)id;
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    LOGWARNING("[vision] vision offline, restart communication.");
}

/* ========================= UART 实现 ========================= */
#ifdef VISION_USE_UART
#include "bsp_usart.h"

/**
 * @brief 接收解包回调（planArray → Vision_Recv_s）
 */
static void DecodeVision(void)
{
    uint8_t *buf = vision_usart_instance->recv_buff;

    // 帧头帧尾检查
    if (buf[0] != 0xAA || buf[VISION_RECV_SIZE - 1] != 0x55) return;

    // 异或校验：索引 1 ~ 19（cmd, len, target_x, target_y, target_yaw, target_time）
    uint8_t xor_calc = 0;
    for (uint8_t i = 1; i < VISION_RECV_SIZE - 2; ++i) xor_calc ^= buf[i];
    if (xor_calc != buf[VISION_RECV_SIZE - 2]) return;

    // 提取规划数据（buf[1]=cmd, buf[2..3]=len，此处固定为20）
    memcpy(&recv_data.target_x,    &buf[4],  4);
    memcpy(&recv_data.target_y,    &buf[8],  4);
    memcpy(&recv_data.target_yaw,  &buf[12], 4);
    memcpy(&recv_data.target_time, &buf[16], 4);

    DaemonReload(vision_daemon_instance);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送机器人状态（robotArray ← Vision_Send_s）
 */
void VisionSend(Vision_Send_s *send)
{
    if (send == NULL) return;

    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len = VISION_SEND_SIZE;

    send_buff[0] = 0xAA;
    send_buff[1] = (uint8_t)send->mode;
    send_buff[2] = (uint8_t)send->state;
    memcpy(&send_buff[3],  &send->robot_x,  4);
    memcpy(&send_buff[7],  &send->robot_y,  4);
    memcpy(&send_buff[11], &send->robot_yaw,4);

    // 异或校验：索引 1 ~ 14
    uint8_t xor_val = 0;
    for (uint8_t i = 1; i < VISION_SEND_SIZE - 2; ++i) xor_val ^= send_buff[i];
    send_buff[VISION_SEND_SIZE - 2] = xor_val;
    send_buff[VISION_SEND_SIZE - 1] = 0x55;

    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_IT);
}
#endif // VISION_USE_UART

/* ========================= VCP 实现 ========================= */
#ifdef VISION_USE_VCP
#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len != VISION_RECV_SIZE) return;
    if (vis_recv_buff[0] != 0xAA || vis_recv_buff[VISION_RECV_SIZE - 1] != 0x55) return;

    uint8_t xor_calc = 0;
    for (uint8_t i = 1; i < VISION_RECV_SIZE - 2; ++i) xor_calc ^= vis_recv_buff[i];
    if (xor_calc != vis_recv_buff[VISION_RECV_SIZE - 2]) return;

    memcpy(&recv_data.target_x,    &vis_recv_buff[4],  4);
    memcpy(&recv_data.target_y,    &vis_recv_buff[8],  4);
    memcpy(&recv_data.target_yaw,  &vis_recv_buff[12], 4);
    memcpy(&recv_data.target_time, &vis_recv_buff[16], 4);

    if (vision_daemon_instance != NULL) DaemonReload(vision_daemon_instance);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    (void)_handle;
    USB_Init_Config_s conf = { .rx_cbk = DecodeVision };
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(Vision_Send_s *send)
{
    if (send == NULL) return;

    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len = VISION_SEND_SIZE;

    send_buff[0] = 0xAA;
    send_buff[1] = (uint8_t)send->mode;
    send_buff[2] = (uint8_t)send->state;
    memcpy(&send_buff[3],  &send->robot_x,  4);
    memcpy(&send_buff[7],  &send->robot_y,  4);
    memcpy(&send_buff[11], &send->robot_yaw,4);

    uint8_t xor_val = 0;
    for (uint8_t i = 1; i < VISION_SEND_SIZE - 2; ++i) xor_val ^= send_buff[i];
    send_buff[VISION_SEND_SIZE - 2] = xor_val;
    send_buff[VISION_SEND_SIZE - 1] = 0x55;

    USBTransmit(send_buff, tx_len);
}
#endif // VISION_USE_VCP

uint8_t VisionIsOnline(void)
{
    return vision_daemon_instance ? DaemonIsOnline(vision_daemon_instance) : 0;
}