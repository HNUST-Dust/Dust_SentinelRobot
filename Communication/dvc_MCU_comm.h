/**
 * @file dvc_MCU_comm.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_COMM_DVC_MCU_COMM_H
#define MODULES_COMM_DVC_MCU_COMM_H

#include "bsp_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

struct McuChassisData
{
    uint8_t          start_of_frame = 0xAA;     // 帧头
    uint16_t         chassis_speed_x;           // 平移方向：左、右
    uint16_t         chassis_speed_y;           // 平移方向：前、后
    uint16_t         chassis_rotation;          // 选装方向：不转、顺时针转、逆时针转
    uint8_t          chassis_spin;              // 小陀螺：不转、顺时针转、逆时针转
};

struct McuCommData
{
    uint8_t         start_of_frame = 0xAB;
    uint8_t         armor;                      // 自瞄
    uint16_t        yaw;                        // yaw
    uint8_t         supercap;                   // 超级电容：充电、放电
    uint8_t         switch_r;
};

struct McuAutoaimData
{
    uint8_t start_of_yaw_frame;
    uint8_t start_of_pitch_frame;
    float yaw_f;
    float pitch_f;
};

class McuComm
{
public:

    volatile McuChassisData mcu_chassis_data_ = {
            0xAA,
            1024,
            1024,
            1024,
            3,
    };
    volatile McuCommData mcu_comm_data_ = {
            0xAB,
            0,
            1024,
            0,
            3
    };

    volatile McuAutoaimData mcu_autoaim_data_ = {0xAC,
                                        0xAD,
                                        0,
                                        0,
                                        };
    void Init(CAN_HandleTypeDef *hcan,
              uint8_t can_rx_id,
              uint8_t can_tx_id);

    void CanRxCpltCallback(uint8_t *rx_data);

    void CanSendCommand();
    void CanSendAutoaim();
    void CanSendNull();

    void Task();


protected:
    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 发送缓冲区
    uint8_t tx_data_[8];
    // 内部函数
    void DataProcess();
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //MODULES_COMM_DVC_MCU_COMM_H
