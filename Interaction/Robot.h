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

#include "alg_pid.h"
#include "bsp_dwt.h"
#include "imu.hpp"

#include "dvc_MCU_comm.h"
#include "dvc_PC_comm.h"

#include "app_gimbal.h"
#include "app_chassis.h"

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
    // 云台保持pid
    Pid yaw_speed_pid_;
    // 云台保持pid
    Pid yaw_angle_pid_;
    // 底盘跟随pid
    Pid chassis_follow_pid_;
    // 底盘陀螺仪
    Imu imu_;


    void Init();
    void Task();
    
protected:
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif