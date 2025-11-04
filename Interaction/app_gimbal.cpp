/**
 * @file app_gimbal.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "app_gimbal.h"
#include "cmsis_os2.h"
#include "stdio.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Gimbal初始化函数
 * 
 */
void Gimbal::Init()
{
    // 4310电机初始化
    motor_yaw_.Init(&hcan1, 0x06, 0x06);

    motor_yaw_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));
    motor_yaw_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    motor_yaw_.SetKd(0); // MIT模式kd
    
    // 小Kp粗调
    // motor_yaw_.SetKp(1.0);  //MIT模式kp
    // motor_yaw_.SetControlAngle(0);
    // motor_yaw_.Output();
    // osDelay(pdMS_TO_TICKS(1000));
    // 大Kp细调
    // motor_yaw_.SetKp(10.0);  //MIT模式kp
    // motor_yaw_.SetControlAngle(1);
    // motor_yaw_.Output();
    // osDelay(pdMS_TO_TICKS(1000));

    // 速度控制
    motor_yaw_.SetKp(0);  //MIT模式kp
    motor_yaw_.SetControlTorque(0);
    motor_yaw_.Output();

    static const osThreadAttr_t kGimbalTaskAttr = 
    {
        .name = "gimbal_task",
        .stack_size = 128 * 6,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Gimbal自身解算函数
 *
 */
void Gimbal::SelfResolution()
{
    now_yaw_angle_   = motor_yaw_.GetNowAngle();
    now_yaw_omega_   = motor_yaw_.GetNowOmega();
    // printf("%f\n", now_yaw_angle_);
    // yaw_angle_pid_.SetNow(now_yaw_angle_);
    // yaw_angle_pid_.CalculatePeriodElapsedCallback();
    // target_yaw_omega_ = yaw_angle_pid_.GetOut();

    // // pitch轴角度归化到±PI / 2之间
    // now_pitch_angle_ = Math_Modulus_Normalization(-motor_pitch_.GetNowAngle(), 2.0f * PI);
}

/**
 * @brief Gimbal输出函数
 *
 */
void Gimbal::Output()
{
    // // 云台位控
    // if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_MANUAL)         // 无自瞄介入
    // {
    //     // do nothing
    // }else if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_AUTOAIM){ // 有自瞄矫正
    //     MotorNearestTransposition();
    //     yaw_angle_pid_.SetTarget(now_yaw_angle_); // 加视觉的相对偏移量
    //     yaw_angle_pid_.CalculatePeriodElapsedCallback();
    //     target_yaw_omega_ = yaw_angle_pid_.GetOut();
    // }

    // motor_yaw_.SetControlOmega(target_yaw_omega_);
    motor_yaw_.SetControlTorque(target_yaw_torque_);
    motor_yaw_.Output();
}

/**
 * @brief Gimbal就近转位函数
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // Yaw就近转位
    float tmp_delta_angle;
    tmp_delta_angle = fmod(target_yaw_angle_ - now_yaw_angle_, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    target_yaw_angle_ = motor_yaw_.GetNowAngle() + tmp_delta_angle;
}

/**
 * @brief Gimbal任务函数
 * 
 */
void Gimbal::Task()
{
    for (;;)
    {
        SelfResolution();
        Output();
        osDelay(pdMS_TO_TICKS(1));
    }
}