/**
 * @file dvc_MCU_comm.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_MCU_comm.h"
#include "dvc_motor_dm.h"
#include "ins_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief MCU通讯函数
 * 
 * @param hcan can句柄
 * @param can_rx_id 接收id
 * @param can_tx_id 发送id
 */
void McuComm::Init(CAN_HandleTypeDef* hcan, uint8_t can_rx_id, uint8_t can_tx_id)
{
     if (hcan->Instance == CAN1)
     {
          can_manage_object_ = &g_can1_manage_object;
     }
     else if (hcan->Instance == CAN2)
     {
          can_manage_object_ = &g_can2_manage_object;
     }

     can_rx_id_ = can_rx_id;
     can_tx_id_ = can_tx_id;

     static const osThreadAttr_t kMcuCommTaskAttr = {
          .name = "mcu_comm_task",
          .stack_size = 512,
          .priority = (osPriority_t) osPriorityNormal
     };
     // 启动任务，将 this 传入
     // osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
}

/**
 * @brief MCU任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void McuComm::TaskEntry(void *argument) {
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

/**
 * @brief MCU任务函数
 * 
 */
void McuComm::Task()
{
     McuCommData send_comm_data_local;
     for (;;)
     {    // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // send_comm_data__Local = *const_cast<const struct McuCommData*>(&(send_comm_data_));
          // __enable_irq();
          
          // 将遥控器数据发给下板
          // CanSendCommand();
          // osDelay(pdMS_TO_TICKS(10));
     }
}

/**
 * @brief MCU can发送底盘数据函数
 * 
 */
void McuComm::CanSendChassis()
{
     static uint8_t can_tx_frame[8];
     // 第一帧发送底盘数据
     can_tx_frame[0] = 0xAA;

     can_tx_frame[1] = send_chassis_data_.chassis_speed_x >> 8;
     can_tx_frame[2] = send_chassis_data_.chassis_speed_x;

     can_tx_frame[3] = send_chassis_data_.chassis_speed_y >> 8;
     can_tx_frame[4] = send_chassis_data_.chassis_speed_y;

     can_tx_frame[5] = send_chassis_data_.rotation >> 8;
     can_tx_frame[6] = send_chassis_data_.rotation;

     can_tx_frame[7] = send_chassis_data_.switch_l;

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief MCU can发送命令数据函数
 * 
 */
void McuComm::CanSendCommand()
{
     static uint8_t can_tx_frame[8];
     union { float f; uint8_t b[4]; } conv;
     conv.f = INS.YawTotalAngle;

     // 第二帧发送通用数据
     can_tx_frame[0] = 0xAB;

     can_tx_frame[1] = send_comm_data_.armor;

     can_tx_frame[2] = send_comm_data_.supercap;

     can_tx_frame[3] = send_comm_data_.switch_r;

     can_tx_frame[4] = conv.b[0];
     can_tx_frame[5] = conv.b[1];
     can_tx_frame[6] = conv.b[2];
     can_tx_frame[7] = conv.b[3];

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief MCU can发送自瞄数据函数
 * 
 */
void McuComm::CanSendAutoaim()
{
     static uint8_t can_tx_frame[8];

     // 第一帧发送yaw包
     can_tx_frame[0] = 0xAC;

     can_tx_frame[1] = send_autoaim_data_.autoaim_yaw[0];
     can_tx_frame[2] = send_autoaim_data_.autoaim_yaw[1];
     can_tx_frame[3] = send_autoaim_data_.autoaim_yaw[2];
     can_tx_frame[4] = send_autoaim_data_.autoaim_yaw[3];
     can_tx_frame[5] = 0x00;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);

     // 第二帧发送pitch包（哨兵pitch角在上板，用不到）
     // can_tx_frame[0] = send_autoaim_data_.start_of_pitch_frame;

     // can_tx_frame[1] = send_autoaim_data_.autoaim_pitch[0];
     // can_tx_frame[2] = send_autoaim_data_.autoaim_pitch[1];
     // can_tx_frame[3] = send_autoaim_data_.autoaim_pitch[2];
     // can_tx_frame[4] = send_autoaim_data_.autoaim_pitch[3];
     
     // can_tx_frame[5] = 0x00;
     // can_tx_frame[6] = 0x00;
     // can_tx_frame[7] = 0x00;
     // can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief MCU can回调函数
 * 
 * @param rx_data 
 */
void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 判断在线

     // 处理数据 , 解包
     switch (rx_data[0])
     {
         case (0xAB):
         {
               union { float f; uint8_t b[4]; } conv;

               conv.b[0]                 = rx_data[4];
               conv.b[1]                 = rx_data[5];
               conv.b[2]                 = rx_data[6];
               conv.b[3]                 = rx_data[7];

               recv_comm_data_.yaw_angle = conv.f;
               break;
         }
     }
}
