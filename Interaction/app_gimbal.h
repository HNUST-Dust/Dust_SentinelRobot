/**
 * @file app_gimbal.h
 * @author qingyu
 * @brief 
 * @version 0.2
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
// module
#include "dvc_motor_dm.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum GimbalControlType
{
    GIMBAL_CONTROL_TYPE_MANUAL = 0,
    GIMBAL_CONTROL_TYPE_AUTOAIM,
};

class Gimbal
{
public:
    // DM4310电机
    MotorDmNormal motor_pitch_;
    // pitch角速度环
    Pid pitch_speed_pid_;
    // pitch角位置环
    Pid pitch_angle_pid_;

    void Init();

    void Task();

    inline float GetNowPitchAngle();

    inline float GetNowPitchOmega();

    inline float GetNowPitchTorque();

    inline float GetTargetPitchAngle();

    inline float GetTargetPitchOmega();

    inline float GetTargetPitchTorque();

    inline void SetTargetPitchAngle(float target_pitch_angle);

    inline void SetTargetPitchOmega(float target_pitch_omega);

    inline void SetTargetPitchTorque(float target_pitch_torque);

protected:
    // pitch轴最小值
    float min_pitch_angle_ = -0.60f;
    // pitch轴最大值
    float max_pitch_angle_ = 0.33f;

    // 内部变量

    // 读变量

    // pitch轴当前角度
    float now_pitch_angle_ = 0.0f;

    // pitch轴当前角速度
    float now_pitch_omega_ = 0.0f;

    // pitch轴当前力矩
    float now_pitch_torque_ = 0.0f;

    // 写变量

    // 云台状态
    GimbalControlType gimbal_control_type_ = GIMBAL_CONTROL_TYPE_MANUAL;
    // 读写变量

    // pitch轴目标角度
    float target_pitch_angle_ = 0.0f;

    // pitch轴目标角速度
    float target_pitch_omega_ = 0.0f;

    // pitch轴目标力矩
    float target_pitch_torque_ = 0.0f;

    void SelfResolution();

    void MotorNearestTransposition();

    void Output();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 获取pitch轴当前角度
 *
 * @return float pitch轴当前角度
 */
inline float Gimbal::GetNowPitchAngle()
{
    return (now_pitch_angle_);
}

/**
 * @brief 获取pitch轴当前角速度
 *
 * @return float pitch轴当前角速度
 */
inline float Gimbal::GetNowPitchOmega()
{
    return (now_pitch_omega_);
}

/**
 * @brief 获取pitch轴当前力矩
 *
 * @return float pitch轴当前力矩
 */
inline float Gimbal::GetNowPitchTorque()
{
    return (now_pitch_torque_);
}

/**
 * @brief 获取pitch轴目标角度
 *
 * @return float pitch轴目标角度
 */
inline float Gimbal::GetTargetPitchAngle()
{
    return (target_pitch_angle_);
}

/**
 * @brief 获取pitch轴目标角速度
 *
 * @return float pitch轴目标角速度
 */
inline float Gimbal::GetTargetPitchOmega()
{
    return (target_pitch_omega_);
}

/**
 * @brief 获取pitch轴目标力矩
 *
 * @return float pitch轴目标力矩
 */
inline float Gimbal::GetTargetPitchTorque()
{
    return (target_pitch_torque_);
}

/**
 * @brief 设定pitch轴角度
 *
 * @param target_pitch_angle pitch轴角度
 */
inline void Gimbal::SetTargetPitchAngle(float target_pitch_angle)
{
    target_pitch_angle_ = target_pitch_angle;
}

/**
 * @brief 设定pitch轴角速度
 *
 * @param target_pitch_omega pitch轴角速度
 */
inline void Gimbal::SetTargetPitchOmega(float target_pitch_omega)
{
    target_pitch_omega_ = target_pitch_omega;
}

/**
 * @brief 设定pitch轴力矩
 *
 * @param target_pitch_torque pitch轴力矩
 */
inline void Gimbal::SetTargetPitchTorque(float target_pitch_torque)
{
    target_pitch_torque_ = target_pitch_torque;
}


#endif // !GIMBAL_H