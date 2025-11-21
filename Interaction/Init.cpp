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
        case (0x030):
        {
            robot_.supercap_.CanRxCpltCallback(CAN_RxMessage->data);
        }
    }
}

/**
 * @brief remote回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_.UartRxCpltCallback(buffer);
}

/**
 * @brief UART6回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart6_callback_function(uint8_t* buffer, uint16_t length)
{	

}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    can_init(&hcan1, can1_callback_function);
    uart_init(&huart6, uart6_callback_function, UART_BUFFER_LENGTH);
    robot_.Init();
}
