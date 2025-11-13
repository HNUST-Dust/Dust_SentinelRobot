/**
 * @file app_reload.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __APP_RELOAD_H__
#define __APP_RELOAD_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Reload
{
public:
    // 拨弹盘1个2006，控制进退弹
    MotorDjiC610 motor_reload_1_;

    void Init();
    void Task();

    inline void SetTargetReloadRotation(float target_reload_rotation);

protected:
    // 目标装载速度 旋转
    float target_reload_rotation_ = 0.0f;

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标装载速度旋转
 * 
 * @param target_reload_rotation 
 */
inline void Reload::SetTargetReloadRotation(float target_reload_rotation)
{
    target_reload_rotation_ = target_reload_rotation;
}


#endif