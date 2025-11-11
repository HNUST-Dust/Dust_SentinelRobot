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
    // 云台初始化
    gimbal_.Init();
    // 底盘陀螺仪初始化
    imu_.Init();
    // 10s时间等待陀螺仪收敛
    osDelay(pdMS_TO_TICKS(10 * 1000));
    // 摩擦轮初始化
    chassis_.Init();
    // 拨弹盘初始化
    reload_.Init();

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
    // Mcu底盘数据
    McuChassisData mcu_chassis_data_local;
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.rotation            = 1024;
    mcu_chassis_data_local.switch_l            = CHASSIS_SPIN_DISABLE;

    // Mcu命令数据
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.armor                  = 0;
    mcu_comm_data_local.supercap               = 0;
    mcu_comm_data_local.switch_r               = Switch_MID;
    mcu_comm_data_local.yaw_angle              = 0;

    // 遥控累加yaw角值
    float yaw_remote_angle = 0.0f;
    // yaw角角度差，用于角度环
    float yaw_angle_diff = 0.0f;
    // 底盘yaw角角度差，用于小陀螺行进
    float chassis_yaw_angle_diff = 0.0f;
    // 底盘yaw角角度差，用于底盘跟随
    float chassis_angle_diff = 0.0f;

    for(;;)
    {
        /****************************   通讯   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *const_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_));
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.recv_comm_data_));
        __enable_irq();


        /****************************   云台   ****************************/


        // 遥控器累加绝对精准yaw轴角度
        yaw_remote_angle += (mcu_chassis_data_local.rotation * K + C) * 1.0;

        // 角度环
        yaw_angle_diff = CalcYawErrorAngle(normalize_angle(mcu_comm_data_local.yaw_angle), normalize_angle(yaw_remote_angle));
        gimbal_.yaw_angle_pid_.SetTarget(0);
        gimbal_.yaw_angle_pid_.SetNow(yaw_angle_diff);
        gimbal_.yaw_angle_pid_.CalculatePeriodElapsedCallback();

        // 速度环
        gimbal_.yaw_speed_pid_.SetTarget(gimbal_.yaw_angle_pid_.GetOut());
        gimbal_.yaw_speed_pid_.SetNow(gimbal_.GetNowYawOmega());
        gimbal_.yaw_speed_pid_.CalculatePeriodElapsedCallback();

        // 发送力矩
        gimbal_.SetTargetYawTorque(gimbal_.yaw_speed_pid_.GetOut());

        // printf("%f\n", gimbal_.yaw_angle_pid_.GetOut());


        /****************************   底盘   ****************************/


        // 计算云台底盘角度差
        chassis_yaw_angle_diff = yaw_remote_angle - imu_.GetYawAngleTotalAngle();
        // 设置当前角度差
        chassis_.SetNowYawAngleDiff(chassis_yaw_angle_diff);
        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((mcu_chassis_data_local.chassis_speed_x * K + C) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((mcu_chassis_data_local.chassis_speed_y * K + C) * MAX_OMEGA_SPEED);


        /****************************   模式   ****************************/


        // 左按钮
        switch (mcu_chassis_data_local.switch_l) 
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
                chassis_angle_diff = CalcYawErrorAngle(normalize_angle(yaw_remote_angle) ,normalize_angle(imu_.GetYawAngleTotalAngle()));

                chassis_.chassis_follow_pid_.SetTarget(0);
                chassis_.chassis_follow_pid_.SetNow(chassis_angle_diff);
                chassis_.chassis_follow_pid_.CalculatePeriodElapsedCallback();

                chassis_.SetTargetVelocityRotation(chassis_.chassis_follow_pid_.GetOut());
                // printf("%f\n", chassis_.chassis_follow_pid_.GetOut());

                break;
            }
            default:
            {
                chassis_.SetTargetVelocityRotation(0);
                gimbal_.SetTargetYawOmega(0);

                break;
            }
        }
        
        // 右按钮
        switch (mcu_comm_data_local.switch_r)
        {
            case Switch_UP:
            {
                reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
                break;
            }
            case Switch_MID:
            {
                reload_.SetTargetReloadRotation(0);
                break;
            }
            case Switch_DOWN:
            {
                reload_.SetTargetReloadRotation(-MAX_RELOAD_SPEED / 2);
                break;
            }
            default:
            {
                reload_.SetTargetReloadRotation(0);
                break;
            }
        }


        /****************************   调试   ****************************/

        osDelay(pdMS_TO_TICKS(1));
    }
}



