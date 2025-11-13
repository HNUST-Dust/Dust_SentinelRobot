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

#define K                       1.f / 660.f
#define C                       -256.f / 165.f
#define MAX_OMEGA_SPEED         25.f
#define MAX_RELOAD_SPEED        -10.f
#define MAX_GYROSCOPE_SPEED     30.f

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
        remote_yaw_angle_ += (mcu_chassis_data_local.rotation * K + C) * 1;

        if(remote_yaw_angle_ >= 180.f){
            remote_yaw_angle_ -= 360.f;
        }else if(remote_yaw_angle_ <= -180.f){
            remote_yaw_angle_ += 360.f;
        }
        gimbal_.SetRemoetYawAngle(remote_yaw_angle_);
        gimbal_.SetImuYawAngle(mcu_comm_data_local.yaw_angle);


        /****************************   底盘   ****************************/


        // 计算云台底盘角度差
        chassis_yaw_angle_diff = normalize_angle(remote_yaw_angle_) - imu_.GetYawAngle();
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
                chassis_.SetTargetVelocityRotation(25);
                break;
            }
            case CHASSIS_SPIN_DISABLE:
            {
                chassis_.SetTargetVelocityRotation(0);
                break;
            }
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
            {
                chassis_angle_diff = CalcYawErrorAngle(normalize_angle(remote_yaw_angle_) ,imu_.GetYawAngle());

                chassis_.chassis_follow_pid_.SetTarget(0);
                chassis_.chassis_follow_pid_.SetNow(chassis_angle_diff);
                chassis_.chassis_follow_pid_.CalculatePeriodElapsedCallback();

                chassis_.SetTargetVelocityRotation(chassis_.chassis_follow_pid_.GetOut());
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

        // printf("%f,%f\n", imu_.GetYawAngleTotalAngle(), mcu_comm_data_local.yaw_angle);
        // printf("%f,%f,%f,%f\n", chassis_.motor_chassis_1_.GetNowOmega(), chassis_.motor_chassis_2_.GetNowOmega(),
        //                         chassis_.motor_chassis_3_.GetNowOmega(), chassis_.motor_chassis_4_.GetNowOmega());
        
        osDelay(pdMS_TO_TICKS(1));
    }
}



