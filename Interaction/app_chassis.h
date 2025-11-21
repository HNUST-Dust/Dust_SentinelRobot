/**
 * @file app_chassis.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
// device
#include "dvc_motor_dji.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Chassis
{
public:
    // 底盘4个3508，控制全向轮
    MotorDjiC620 motor_chassis_1_,
                 motor_chassis_2_,
                 motor_chassis_3_,
                 motor_chassis_4_;

    // 底盘跟随pid
    Pid chassis_follow_pid_;

    void Init();

    void Task();

    inline void SetTargetVxInGimbal(float target_velocity_x);

    inline void SetTargetVyInGimbal(float target_velocity_y);

    inline void SetTargetVelocityRotation(float target_velocity_rotation);

    inline void SetNowYawAngleDiff(float now_yawdiff);

    inline void SetMaxAccelXY(float max_accel_xy);

    inline void SetMaxAccelR(float max_accel_r);

protected:
// 底盘速度参数

    // 云台坐标系目标速度

    float target_vx_in_gimbal_ = 0.0f;
    
    float target_vy_in_gimbal_ = 0.0f;

    // 底盘坐标系目标速度

    float target_vx_in_chassis_ = 0.0f;

    float target_vy_in_chassis_ = 0.0f;

    // 目标速度 旋转

    float target_velocity_rotation_ = 0.0f;

// 旋转矩阵参数

    float now_yawdiff_ = 0.0f;

    float cos_theta_ = 1.0f;
    
    float sin_theta_ = 0.0f;

// 斜坡规划参数

    // xyr轴最大加速度

    float max_accel_xy_ = 150.f;

    float max_accel_r_  = 150.f;

    // xyr轴最大减速度
    float max_decel_xy_ = -200.f;

    float max_decel_r_  = -200.f;

    // xyr当前加速度

    float now_accel_x_ = 0.0f;

    float now_accel_y_ = 0.0f;

    float now_accel_r_ = 0.0f;

    // xyr上一次目标速度

    float last_target_vx_ = 0.0f;

    float last_target_vy_ = 0.0f;

    float last_target_r_ = 0.0f;

    // 底盘频率

    float dt = 0.001;

// 底盘驱动

    void KinematicsInverseResolution();

    void SlopePlanning();

    void RotationMatrixTransform();

    void OutputToMotor();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标速度X
 *
 * @param target_velocity_x 目标速度X
 */
inline void Chassis::SetTargetVxInGimbal(float target_vx_in_gimbal)
{
    target_vx_in_gimbal_ = target_vx_in_gimbal;
}

/**
 * @brief 设定目标速度Y
 *
 * @param target_velocity_y 目标速度Y
 */
inline void Chassis::SetTargetVyInGimbal(float target_vy_in_gimbal)
{
    target_vy_in_gimbal_ = target_vy_in_gimbal;
}

/**
 * @brief 设定目标速度旋转
 *
 * @param target_velocity_rotation 目标速度Y
 */
inline void Chassis::SetTargetVelocityRotation(float target_velocity_rotation)
{
    target_velocity_rotation_ = target_velocity_rotation;
}

/**
 * @brief 设定现在yaw角偏差
 * 
 * @param now_yawdiff 
 */
inline void Chassis::SetNowYawAngleDiff(float now_yawdiff)
{
    now_yawdiff_ = now_yawdiff;
}

inline void Chassis::SetMaxAccelXY(float max_accel_xy)
{
    max_accel_xy_ = max_accel_xy;
}

inline void Chassis::SetMaxAccelR(float max_accel_r)
{
    max_accel_r = max_accel_r;
}

#endif // !APP_CHASSIS_H_