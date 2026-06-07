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
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

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

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
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
}

/**
 * YYP0418修改
 * @brief 改为根据底盘IMU获取的当前底盘角度计算和目标保持角度的误差,底盘保持角度由遥控器右侧开关控制
 * @brief (旧)根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo 将占位的angle修改为底盘IMU获取的当前底盘角度减去应保持的角度,并调整计算方式以适应新的输入值
 */
static void CalcOffsetAngle()
{
    
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    // angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    angle = 0; // 占位，之后修改为底盘IMU获取的当前底盘角度减去应保持的角度
    chassis_cmd_send.offset_angle = angle;
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #endif
}

/**
 * YYP0418修改
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 左侧开关状态[下],底盘保持前向
    {
        chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 左侧开关状态[上],底盘自由旋转移动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = (float)rc_data[TEMP].rc.rocker_l_*3; // 设置底盘旋转速度,增益系数需要调整
        // gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    // 自动接球模式
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[下],视觉模式
    {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    // { // 按照摇杆的输出大小进行角度增量,增益系数需调整
    //     gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    //     gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    // }


    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    if(abs(rc_data[TEMP].rc.rocker_r_)>50)
        chassis_cmd_send.vx = 15.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    else
        chassis_cmd_send.vx = 0;
    if(abs(rc_data[TEMP].rc.rocker_r1)>50)
        chassis_cmd_send.vy = 15.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向
    else
        chassis_cmd_send.vy = 0;

    //YYP0417修改：根据遥控器右侧开关的状态切换发球杆状态,右侧开关[上]为零位,右侧开关[下]为打出
    if (switch_is_up(rc_data[TEMP].rc.switch_right)&&g_launcher_status!=LAUNCHER_STOP) // 右侧开关状态[上],发球杆在初始位
        g_launcher_status=LAUNCHER_ORIGIN;                                            
    else if(g_launcher_status!=LAUNCHER_STOP)
        g_launcher_status=LAUNCHER_HIT; /// 右侧开关状态[下],发球杆打出


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
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
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
    // 纯遥控器
    RemoteControlSet();


    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

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
