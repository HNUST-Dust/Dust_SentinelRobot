/**
 * @file Init.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Init.h"
#include "Robot.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "usart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

Robot robot_;

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief can1回调函数
 * 
 * @param CAN_RxMessage 
 */
void can1_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x01):
        {
            robot_.mcu_comm_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x06):
        {
            robot_.gimbal_.motor_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        } 
        case (0x201):
        {
            robot_.chassis_.motor_chassis_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot_.chassis_.motor_chassis_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            robot_.chassis_.motor_chassis_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            robot_.chassis_.motor_chassis_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x205):
        {
            robot_.reload_.motor_reload_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    can_init(&hcan1, can1_callback_function);
    robot_.Init();
}
