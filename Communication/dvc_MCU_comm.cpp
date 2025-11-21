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
     for (;;)
     {    // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // mcu_comm_data_local = *const_cast<const struct McuCommData*>(&(mcu_comm_data_));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}

/**
 * @brief MCU can发送命令函数
 * 
 */
void McuComm::CanSendCommand()
{
     
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
          case (0xAA): // 底盘包
          {
               // recv_chassis_data_.start_of_frame       = rx_data[0];
               recv_chassis_data_.chassis_speed_x      = rx_data[1] << 8 | rx_data[2];
               recv_chassis_data_.chassis_speed_y      = rx_data[3] << 8 | rx_data[4];
               recv_chassis_data_.rotation             = rx_data[5] << 8 | rx_data[6];

               switch(rx_data[7])
               {
                    case 1:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_CLOCKWISE;
                    break;
                    case 3:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_DISABLE;
                    break;
                    case 2:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_COUNTER_CLOCK_WISE;
                    break;
                    default:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_DISABLE;
                    break;
               }
               
               break;
          }
          case (0xAB): // 拨弹盘，yaw角包
          {
               union { float f; uint8_t b[4]; } conv;

               // recv_comm_data_.start_of_frame       = rx_data[0];
               recv_comm_data_.armor                = rx_data[1];
               recv_comm_data_.supercap             = rx_data[2];

               switch(rx_data[3])
               {
                    case 1:
                    recv_comm_data_.switch_r = Switch_UP;
                    break;
                    case 3:
                    recv_comm_data_.switch_r = Switch_MID;
                    break;
                    case 2:
                    recv_comm_data_.switch_r = Switch_DOWN;
                    break;
                    default:
                    recv_comm_data_.switch_r = Switch_MID;
                    break;
               }

               conv.b[0]                           = rx_data[4];
               conv.b[1]                           = rx_data[5];
               conv.b[2]                           = rx_data[6];
               conv.b[3]                           = rx_data[7];
               
               recv_comm_data_.yaw_angle = conv.f;

               break;
          }
          case (0xAC): // 自瞄yaw包
          {
               union { float f; uint8_t b[4]; } conv;

               // recv_autoaim_data_.start_of_yaw_frame = rx_data[0];

               conv.b[0]     = rx_data[1];
               conv.b[1]     = rx_data[2];
               conv.b[2]     = rx_data[3];
               conv.b[3]     = rx_data[4];

               recv_autoaim_data_.autoaim_yaw = conv.f;

               break;
          }
     }

}
