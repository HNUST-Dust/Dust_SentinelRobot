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

#include "bsp_dwt.h"
#include "imu.hpp"

#include "dvc_remote_dji.h"
#include "dvc_MCU_comm.h"
#include "dvc_PC_comm.h"
#include "VT03.h"

#include "app_gimbal.h"
#include "app_shoot.h"

#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Robot
{
public:
    // 遥控
    RemoteDjiDR16 remote_dr16_;
    
    VT03 remote_vt03_;
    // 上下板通讯
    McuComm mcu_comm_;
    // pitch角云台
    Gimbal gimbal_;
    // 摩擦轮
    Shoot shoot_;
    // 上位机通讯
    PcComm pc_comm_;
    // 云台陀螺仪
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