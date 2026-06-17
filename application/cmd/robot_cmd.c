// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "optical_flow.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// Yaw数据源选择: 0=DM_IMU(CAN), 1=BMI088+INS(SPI/EKF)
#define YAW_SOURCE_DM_IMU     0
#define YAW_SOURCE_BMI088_INS 1
#define CHASSIS_YAW_SOURCE    YAW_SOURCE_BMI088_INS

float temp_float=0;
float temp_float1=0;
//YYP0417添加：发球杆状态全局变量定义,初始状态设为零位,根据遥控器右侧开关的状态进行切换
LauncherStatus_TypeDef g_launcher_status = LAUNCHER_ORIGIN;

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static OpticalFlowInstance *optical_flow;  // 光流模块实例
static attitude_t *ins_imu_data;        // BMI088 INS 解算结果指针
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

// static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
// static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
// static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
// static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

// static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
// static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
// static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
// static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口huart

    // gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    // gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

    DM_IMU_Init_Config_s imu_conf = {
        .tx_id = 0x11,        // MCU → IMU 指令
        .rx_id = 0x01,        // IMU → MCU 欧拉角
        .can_handle = &hfdcan3,
    };
    DM_IMU_Init(&imu_conf);

    OpticalFlow_Init_Config_s flow_conf = {
        .usart_handle = &huart7,
        .protocol = OPTICAL_FLOW_UPIXELS,
        .flow_scale = OPTICAL_FLOW_DEFAULT_SCALE,
        .enable_global_frame = 1,  /* 使能偏航角世界坐标系映射 */
        .y_direction = -1,         /* Y 轴反向 */
    };
    optical_flow = OpticalFlowInit(&flow_conf);

    ins_imu_data = INS_Init(); // 获取 BMI088 EKF 解算结果指针(幂等,可安全多次调用)
}

/**
 * YYP0418修改
 * @brief 根据IMU获取的当前底盘yaw角计算和目标保持角度的误差
 *        进入CHASSIS_KEEP_FRONT模式时捕获当前yaw作为目标角度,
 *        之后每周期计算 yaw - keep_angle 并归一化到[-180, 180]
 *
 *        yaw数据源由CHASSIS_YAW_SOURCE宏控制:
 *        YAW_SOURCE_DM_IMU → DM-IMU-L1 (CAN, Euler)
 *        YAW_SOURCE_BMI088_INS → BMI088 + EKF (SPI)
 */
static void CalcOffsetAngle()
{
    static float keep_angle = 0;
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;

    float yaw;
#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    const DM_IMU_Data_s *imu = DM_IMU_GetData();
    yaw = imu->yaw;
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    yaw = ins_imu_data->Yaw;
#else
    yaw = 0;
#endif

    // 进入KEEP_FRONT模式时锁定当前yaw为目标角度
    if (chassis_cmd_send.chassis_mode == CHASSIS_KEEP_FRONT
        && last_mode != CHASSIS_KEEP_FRONT)
    {
        keep_angle = yaw;
    }
    last_mode = chassis_cmd_send.chassis_mode;

    // 计算偏移并归一化到[-180, 180]
    float offset = yaw - keep_angle;
    if (offset > 180.0f)  offset -= 360.0f;
    if (offset < -180.0f) offset += 360.0f;
    chassis_cmd_send.offset_angle = offset;
}

/**
 * @brief 视觉导航: 替换摇杆 vx/vy 为导航计算值.
 *
 * 不动条件: 视觉离线 | 光流离线 | target=(0,0) | 已到达目标.\n
 * 右开关及朝向控制逻辑与手动模式一致, 本函数不干预.
 */
static void AutoNavigation(void)
{
    if (!VisionIsOnline() || !OpticalFlowIsOnline(optical_flow)
        || (vision_recv_data->target_x == 0.0f && vision_recv_data->target_y == 0.0f))
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        return;
    }

    const OpticalFlow_Data_s *flow_data = OpticalFlowGetData(optical_flow);
    float err_x = vision_recv_data->target_x - flow_data->position_x_global;
    float err_y = vision_recv_data->target_y - flow_data->position_y_global;
    float dist = Sqrt(err_x * err_x + err_y * err_y);

    if (dist < NAV_ARRIVAL_DIST)
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
    }
    else
    {
        float speed = dist * NAV_SPEED_GAIN;
        speed = float_constrain(speed, 0.0f, NAV_MAX_SPEED);
        chassis_cmd_send.vx = (err_x / dist) * speed;
        chassis_cmd_send.vy = (err_y / dist) * speed;
    }
}

/**
 * YYP0418修改
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    /* ===== 左开关: vx/vy 来源 ===== */
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        /* 手动模式: vx/vy 来自摇杆 */
        if (abs(rc_data[TEMP].rc.rocker_r_) > 50)
            chassis_cmd_send.vx = 30.0f * (float)rc_data[TEMP].rc.rocker_r_;
        else
            chassis_cmd_send.vx = 0;
        if (abs(rc_data[TEMP].rc.rocker_r1) > 50)
            chassis_cmd_send.vy = 30.0f * (float)rc_data[TEMP].rc.rocker_r1;
        else
            chassis_cmd_send.vy = 0;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        /* 自动模式: vx/vy 来自视觉导航 */
        AutoNavigation();
    }

    /* ===== 右开关: 朝向模式, 手动/自动共用 ===== */
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = (float)rc_data[TEMP].rc.rocker_l_ * 4;
    }

    /* ===== 发球杆控制 ===== */
    //YYP0417修改：根据遥控器右侧开关的状态切换发球杆状态,右侧开关[上]为零位,右侧开关[下]为打出
    if (switch_is_up(rc_data[TEMP].rc.switch_right) && g_launcher_status != LAUNCHER_STOP)
        g_launcher_status = LAUNCHER_ORIGIN;
    else if (g_launcher_status != LAUNCHER_STOP)
        g_launcher_status = LAUNCHER_HIT;
}

/**
 *   YYP0418删除
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
        // 占位
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        // shoot_cmd_send.shoot_mode = SHOOT_OFF;
        // shoot_cmd_send.friction_mode = FRICTION_OFF;
        // shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) 
 * YYP0418修改*/
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    // SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    // SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 计算偏移角度,仅在右侧开关状态为[下]需要保持前向时使用
    CalcOffsetAngle();

    // 更新光流模块偏航角,用于世界坐标系旋转映射
#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    OpticalFlowSetYaw(optical_flow, DM_IMU_GetData()->yaw);
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    OpticalFlowSetYaw(optical_flow, ins_imu_data->Yaw);
#endif

    // 读取光流模块累计位移数据,使用世界坐标系下的值
    const OpticalFlow_Data_s *flow_data = OpticalFlowGetData(optical_flow);
    if (flow_data->updated) {
        OpticalFlowClearUpdated(optical_flow);
        temp_float_x = flow_data->position_x_global;
        temp_float_y=flow_data->position_y_global;
    }

    // 纯遥控器
    RemoteControlSet();


    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 上报世界坐标系位置及偏航角到视觉上位机
#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    vision_send_data.robot_yaw = DM_IMU_GetData()->yaw;
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    vision_send_data.robot_yaw = ins_imu_data->Yaw;
#endif
    vision_send_data.robot_x = flow_data->position_x_global;
    vision_send_data.robot_y = flow_data->position_y_global;

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    // PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    // PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);//发送
}
