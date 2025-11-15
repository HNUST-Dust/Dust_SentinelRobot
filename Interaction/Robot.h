/**
 * @file app_robot.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ROBOT_H__
#define __ROBOT_H__

/* Includes ------------------------------------------------------------------*/

// alg
#include "alg_pid.h"
// bsp
#include "stdio.h"
#include "bsp_dwt.h"
// dvc
#include "dvc_MCU_comm.h"
#include "dvc_PC_comm.h"
#include "imu.hpp"
// app
#include "app_reload.h"
#include "app_gimbal.h"
#include "app_chassis.h"
#include "supercap.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Robot
{
public:
    // 上下板通信
    McuComm mcu_comm_;
    // 底盘驱动
    Chassis chassis_;
    // yaw角云台
    Gimbal gimbal_;
    // 拨弹盘
    Reload reload_;
    // 底盘陀螺仪
    Imu imu_;
    // 超级电容
    Supercap supercap_;

    void Init();

    void Task();
    
protected:

    // 遥控累加yaw角值
    float remote_yaw_angle_ = 0.0f;

    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif