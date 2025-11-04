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

/* Private macros ------------------------------------------------------------*/

#define K                    1.f / 660.f
#define C                    -256.f / 165.f
#define MAX_OMEGA_SPEED      15.f
#define MAX_YAW_SPEED        10.f
#define MAX_RELOAD_SPEED     -10.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    dwt_init(168);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x01, 0x00);
    // yaw角角度环pid
    yaw_angle_pid_.Init(
        0.47f,
        0.002f,
        0.00075f,
        0.0f,
        0.f,
        15.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f  
    );
    // yaw角速度环pid
    yaw_speed_pid_.Init(
        0.725f,
        0.0002f,
        0.0f,
        0.0f,
        0.0f,
        10.f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f  
    );
    // 云台初始化
    gimbal_.Init();
    // 底盘陀螺仪初始化
    imu_.Init();
    // 10s时间等待陀螺仪收敛
    // osDelay(pdMS_TO_TICKS(7 * 1000));
    // 摩擦轮初始化
    chassis_.Init();

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
    McuChassisData mcu_chassis_data_local;
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.chassis_rotation    = 1024;
    mcu_chassis_data_local.chassis_spin        = CHASSIS_SPIN_DISABLE;

    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.armor                  = 0;
    mcu_comm_data_local.supercap               = 0;
    mcu_comm_data_local.switch_r               = Switch_MID;
    mcu_comm_data_local.yaw                    = 1024;

    McuAutoaimData mcu_autoaim_data_local;
    mcu_autoaim_data_local.yaw_f               = 0;

    float yaw_remote_angle = 0.0f;

    for(;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *const_cast<const McuChassisData*>(&(mcu_comm_.mcu_chassis_data_));
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        mcu_autoaim_data_local = *const_cast<const McuAutoaimData*>(&(mcu_comm_.mcu_autoaim_data_));
        __enable_irq();

        //***************************   云台   ***************************//
        yaw_remote_angle += (mcu_comm_data_local.yaw * K + C) * 1.0;

        yaw_angle_pid_.SetTarget(yaw_remote_angle);
        yaw_angle_pid_.SetNow(mcu_autoaim_data_local.yaw_f);
        yaw_angle_pid_.CalculatePeriodElapsedCallback();

        yaw_speed_pid_.SetTarget(yaw_angle_pid_.GetOut());
        yaw_speed_pid_.SetNow(gimbal_.GetNowYawOmega());
        yaw_speed_pid_.CalculatePeriodElapsedCallback();
        gimbal_.SetTargetYawTorque(yaw_speed_pid_.GetOut());

        //***************************   底盘   ***************************//

        chassis_.SetNowYawAngleDiff(yaw_remote_angle - imu_.GetYawAngleTotalAngle());

        chassis_.SetTargetVxInGimbal((mcu_chassis_data_local.chassis_speed_x * K + C) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((mcu_chassis_data_local.chassis_speed_y * K + C) * MAX_OMEGA_SPEED);

        switch (mcu_chassis_data_local.chassis_spin) 
        {
            case CHASSIS_SPIN_CLOCKWISE:
            {
                chassis_.SetTargetVelocityRotation(MAX_OMEGA_SPEED);
                break;
            }
            case CHASSIS_SPIN_DISABLE:
            {
                chassis_.SetTargetVelocityRotation(0);
                break;
            }
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
            {
                chassis_.SetTargetVelocityRotation((mcu_chassis_data_local.chassis_rotation * K + C) * MAX_OMEGA_SPEED);
                break;
            }
            default:
            {
                chassis_.SetTargetVelocityRotation((mcu_chassis_data_local.chassis_rotation * K + C) * MAX_OMEGA_SPEED);
                gimbal_.SetTargetYawOmega(0);
                break;
            }
        }
        switch (mcu_comm_data_local.switch_r)
        {
            case Switch_UP:
            {
                chassis_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
                break;
            }
            case Switch_MID:
            {
                chassis_.SetTargetReloadRotation(0);
                break;
            }
            case Switch_DOWN:
            {
                chassis_.SetTargetReloadRotation(-MAX_RELOAD_SPEED / 4);
                break;
            }
            default:
            {
                chassis_.SetTargetReloadRotation(0);
                break;
            }
        }
        osDelay(pdMS_TO_TICKS(1));
    }
}



