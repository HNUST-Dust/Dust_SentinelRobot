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
    // yaw轴角度环pid
    yaw_angle_pid_.Init(
        0.5f,
        0.05f,
        0.0105f,
        0.0f,
        0.f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        PID_D_First_DISABLE,
        0.01
    );
    // yaw轴速度环pid
    yaw_omega_pid_.Init(
        0.4f,
        0.008f,
        0.0f,
        0.0f,
        0.0f,
        9.9f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        PID_D_First_DISABLE,
        0.01
         
    );
    // yaw轴速度环滤波器
    yaw_omega_filter_.Init(15.0f, 0.001f);

    // 4310电机初始化
    motor_yaw_.Init(&hcan1, 0x06, 0x06);

    // 发送清除错误指令
    motor_yaw_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));
    
    // 保存零点（当云台与底盘上电有偏差时需重新设置零点）
    // motor_yaw_.CanSendSaveZero();
    // osDelay(pdMS_TO_TICKS(1000));
    // 发送使能命令
    motor_yaw_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    // 小Kp粗调
    motor_yaw_.SetKp(0.8);    // MIT模式kp
    motor_yaw_.SetKd(0.3);    // MIT模式kd
    motor_yaw_.SetControlAngle(0);
    motor_yaw_.Output();
    osDelay(pdMS_TO_TICKS(1300));
    // 大Kp细调
    motor_yaw_.SetKp(10.0);    // MIT模式kp
    motor_yaw_.SetKd(1.0);    // MIT模式kd
    motor_yaw_.SetControlAngle(0);
    motor_yaw_.Output();
    osDelay(pdMS_TO_TICKS(500));

    // 力矩控制
    motor_yaw_.SetKp(0);  // MIT模式kp
    motor_yaw_.SetKd(0);  // MIT模式kd
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
    // 获取当前数据
    now_yaw_omega_ = motor_yaw_.GetNowOmega();
    now_yaw_angle_ = motor_yaw_.GetNowAngle() * 14.4;

    // 计算yaw轴偏差
    // yaw_angle_diff_ = CalcYawErrorAngle(imu_yaw_angle_, remote_yaw_angle_);
    yaw_angle_diff_ = CalcYawErrorAngle(now_yaw_angle_, remote_yaw_angle_);


    // 角度环
    yaw_angle_pid_.SetTarget(0);
    yaw_angle_pid_.SetNow(yaw_angle_diff_);
    yaw_angle_pid_.CalculatePeriodElapsedCallback();

    // 速度环
    yaw_omega_pid_.SetTarget(yaw_angle_pid_.GetOut());
    float filtered_omega = yaw_omega_filter_.Update(now_yaw_omega_);    // 一阶低通滤波
    yaw_omega_pid_.SetNow(filtered_omega);
    yaw_omega_pid_.CalculatePeriodElapsedCallback();

    // 设定目标力矩
    // SetTargetYawTorque(yaw_omega_pid_.GetOut());
    // printf("%f,%f,%f\n", yaw_angle_diff_, yaw_angle_pid_.GetOut(), yaw_omega_pid_.GetOut());
}

/**
 * @brief Gimbal输出函数
 *
 */
void Gimbal::Output()
{
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
