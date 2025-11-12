/**
 * @file app_robot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Robot.h"
#include "bsp_uart.h"
#include "usart.h"

/* Private macros ------------------------------------------------------------*/

#define MAX_SHOOT_SPEED             50.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    // 上位机通讯
    pc_comm_.Init();
    // 遥控初始化
    // remote_vt03_.Init(&huart1, uart1_callback_function, UART_BUFFER_LENGTH);

    remote_dr16_.Init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x00, 0x01);
    // 等待云台yaw角回正
    osDelay(pdMS_TO_TICKS(5000));
    // 底盘陀螺仪初始化
    imu_.Init();
    // 10s时间等待陀螺仪收敛

    // 云台初始化
    gimbal_.Init();
    // 摩擦轮初始化
    shoot_.Init();
    
    static const osThreadAttr_t kRobotTaskAttr = 
    {
        .name = "robot_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    for(;;)
    {
        /****************************   通讯   ****************************/


        // 若掉线发送空白数据
        if(remote_dr16_.remote_dji_alive_status == REMOTE_DJI_STATUS_DISABLE)
        {
            mcu_comm_.DisconnectData();
        }
        // 发送下板数据
        mcu_comm_.CanSendChassis();
        mcu_comm_.CanSendCommand();


        /****************************   云台   ****************************/

        gimbal_.SetRemoetPitchAngle(remote_dr16_.output_.pitch);

        /****************************   模式   ****************************/


        // 右按钮
        switch (remote_dr16_.output_.switch_r)
        {
            case SWITCH_UP:
            {
                shoot_.SetTargetShootSpeed(MAX_SHOOT_SPEED);
                break;
            }
            case SWITCH_MID:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
            default:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
        }


        /****************************   调试   ****************************/


        // pc_comm_.send_autoaim_data.armor = 0x00;
        // pc_comm_.send_autoaim_data.yaw   =  0.f;
        // pc_comm_.send_autoaim_data.pitch =  0.f;
        // pc_comm_.Send_Message();

        // memcpy(mcu_comm_.send_autoaim_data_.autoaim_yaw, pc_comm_.recv_autoaim_data.yaw, 4);
        // mcu_comm_.CanSendAutoaim();

        // printf("%f\n", normalize_angle(imu_.GetYawAngleTotalAngle()));

        // printf("%f\n", imu_.GetTemperature());

        osDelay(pdMS_TO_TICKS(1));
    }
}



