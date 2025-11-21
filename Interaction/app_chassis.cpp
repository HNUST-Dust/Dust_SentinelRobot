/**
 * @file app_chassis.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "app_chassis.h"
#include "dvc_motor_dji.h"
#include "stdio.h"
#include <stdint.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Chassis初始化函数
 * 
 */
void Chassis::Init()
{
    // 底盘跟随pid
    chassis_follow_pid_.Init(
        0.3f,
        0.01f,
        0.002f,
        0.0f,
        0.0f,
        25.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f  
    );
    // 底盘3508电机初始化
    motor_chassis_1_.pid_omega_.Init(0.15f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    motor_chassis_2_.pid_omega_.Init(2.0f,0.f,0.0007f);
    motor_chassis_3_.pid_omega_.Init(2.0f,0.f,0.0007f);
    motor_chassis_4_.pid_omega_.Init(2.0f,0.f,0.0007f);

    motor_chassis_1_.Init(&hcan1, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_2_.Init(&hcan1, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_3_.Init(&hcan1, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_4_.Init(&hcan1, MOTOR_DJI_ID_0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    motor_chassis_1_.SetTargetOmega(0.0f);
    motor_chassis_2_.SetTargetOmega(0.0f);
    motor_chassis_3_.SetTargetOmega(0.0f);
    motor_chassis_4_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kChassisTaskAttr = 
    {
        .name = "chassis_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Chassis::TaskEntry, this, &kChassisTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Chassis::TaskEntry(void *argument)
{
    Chassis *self = static_cast<Chassis *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Chassis旋转矩阵变换
 * 
 */
void Chassis::RotationMatrixTransform()
{
    cos_theta_ = cosf(now_yawdiff_ * PI / 180.f);
    sin_theta_ = sinf(now_yawdiff_ * PI / 180.f);
    target_vx_in_chassis_ = cos_theta_ * target_vx_in_gimbal_ - sin_theta_ * target_vy_in_gimbal_;
    target_vy_in_chassis_ = sin_theta_ * target_vx_in_gimbal_ + cos_theta_ * target_vy_in_gimbal_;
}

/**
 * @brief Chassis斜坡规划
 * 
 */
void Chassis::SlopePlanning()
{
    now_accel_x_ = (target_vx_in_chassis_ - last_target_vx_) / dt;
    now_accel_y_ = (target_vy_in_chassis_ - last_target_vy_) / dt;
    now_accel_r_ = (target_velocity_rotation_ - last_target_r_) / dt;

    if(fabs(now_accel_x_) > max_accel_xy_)
    {
        if(target_vx_in_chassis_ > last_target_vx_)
        {
            target_vx_in_chassis_ = (last_target_vx_ + max_accel_xy_ * dt) < target_vx_in_chassis_ ? 
                                    (last_target_vx_ + max_accel_xy_ * dt) : target_vx_in_chassis_;
        } 
        else
        {
            target_vx_in_chassis_ = (last_target_vx_ - max_accel_xy_ * dt) > target_vx_in_chassis_ ? 
                                    (last_target_vx_ - max_accel_xy_ * dt) : target_vx_in_chassis_;
        }
    }

    if(fabs(now_accel_y_) > max_accel_xy_)
    {
        if(target_vy_in_chassis_ > last_target_vy_) 
        {
            target_vy_in_chassis_ = (last_target_vy_ + max_accel_xy_ * dt) < target_vy_in_chassis_ ? 
                                    (last_target_vy_ + max_accel_xy_ * dt) : target_vy_in_chassis_;
        } 
        else
        {
            target_vy_in_chassis_ = (last_target_vy_ - max_accel_xy_ * dt) > target_vy_in_chassis_ ? 
                                    (last_target_vy_ - max_accel_xy_ * dt) : target_vy_in_chassis_;
        }
    }

    if(fabs(now_accel_r_) > max_accel_r_)
    {
        if(target_velocity_rotation_ > last_target_r_) 
        {
            target_velocity_rotation_ = (last_target_r_ + max_accel_r_ * dt) < target_velocity_rotation_ ? 
                                        (last_target_r_ + max_accel_r_ * dt) : target_velocity_rotation_;
        } 
        else
        {
            target_velocity_rotation_ = (last_target_r_ - max_accel_r_ * dt) > target_velocity_rotation_ ? 
                                        (last_target_r_ - max_accel_r_ * dt) : target_velocity_rotation_;
        }
    }

    last_target_vx_ = target_vx_in_chassis_;
    last_target_vy_ = target_vy_in_chassis_;
    last_target_r_  = target_velocity_rotation_;
}

/**
 * @brief Chassis运动学解析
 * 
 */
void Chassis::KinematicsInverseResolution()
{
    motor_chassis_1_.SetTargetOmega( target_vx_in_chassis_ - target_vy_in_chassis_ + target_velocity_rotation_);
    motor_chassis_2_.SetTargetOmega(-target_vx_in_chassis_ - target_vy_in_chassis_ + target_velocity_rotation_);
    motor_chassis_3_.SetTargetOmega(-target_vx_in_chassis_ + target_vy_in_chassis_ + target_velocity_rotation_);
    motor_chassis_4_.SetTargetOmega( target_vx_in_chassis_ + target_vy_in_chassis_ + target_velocity_rotation_);
}

/**
 * @brief Chassis电机输出
 * 
 */
void Chassis::OutputToMotor()
{
    motor_chassis_1_.CalculatePeriodElapsedCallback();
    motor_chassis_2_.CalculatePeriodElapsedCallback();
    motor_chassis_3_.CalculatePeriodElapsedCallback();
    motor_chassis_4_.CalculatePeriodElapsedCallback();

    // 全向轮底盘电机
    can_send_data(&hcan1, 0x200, g_can1_0x200_tx_data, 8);
}


/**
 * @brief Chassis任务函数
 * 
 */
void Chassis::Task()
{
    for (;;)
    {
        // // 旋转矩阵转换
        // RotationMatrixTransform();
        // // 斜坡规划算法
        // SlopePlanning();
        // // 运动学解析
        // KinematicsInverseResolution();
        // // 输出
        // OutputToMotor();
        
        motor_chassis_1_.CalculatePeriodElapsedCallback();
        can_send_data(&hcan1, 0x200, g_can1_0x200_tx_data, 8);
        
        osDelay(pdMS_TO_TICKS(1));
    }
}

