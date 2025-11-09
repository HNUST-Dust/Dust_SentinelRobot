/**
 * @file app_reload.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "app_reload.h"
#include "stdio.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Reload初始化函数
 * 
 */
void Reload::Init()
{
    // 拨弹盘2006电机初始化
    motor_reload_1_.pid_omega_.Init(0.5f, 0.0f, 0.0f);
    
    motor_reload_1_.Init(&hcan1, MOTOR_DJI_ID_0x205, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    
    motor_reload_1_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kReloadTaskAttr = 
    {
        .name = "reload_task",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Reload::TaskEntry, this, &kReloadTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Reload::TaskEntry(void *argument)
{
    Reload *self = static_cast<Reload *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Reload任务函数
 * 
 */
void Reload::Task()
{
    for(;;)
    {
        // 设置拨弹速度
        motor_reload_1_.SetTargetOmega( target_reload_rotation_);

        motor_reload_1_.CalculatePeriodElapsedCallback();

        can_send_data(&hcan1, 0x1FF, g_can1_0x1ff_tx_data, 2);

        osDelay(pdMS_TO_TICKS(10));
    }
}
