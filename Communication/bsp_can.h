/**
 * @file bsp_can.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_CAN_H_
#define BSP_CAN_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "can.h"

/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct CanRxBuffer
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CanCallback)(struct CanRxBuffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct CanManageObject
{
    CAN_HandleTypeDef *can_handler;
    struct CanRxBuffer rx_buffer;
    CanCallback callback_function;
};

/* Exported variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef g_hfdcan1;
extern CAN_HandleTypeDef g_hfdcan2;
extern CAN_HandleTypeDef g_hfdcan3;

extern struct CanManageObject g_can1_manage_object;
extern struct CanManageObject g_can2_manage_object;
extern struct CanManageObject g_can3_manage_object;

extern uint8_t g_can1_0x1ff_tx_data[];
extern uint8_t g_can1_0x200_tx_data[];
extern uint8_t g_can1_0x2ff_tx_data[];
extern uint8_t g_can1_0x3fe_tx_data[];
extern uint8_t g_can1_0x4fe_tx_data[];

extern uint8_t g_can2_0x1ff_tx_data[];
extern uint8_t g_can2_0x200_tx_data[];
extern uint8_t g_can2_0x2ff_tx_data[];
extern uint8_t g_can2_0x3fe_tx_data[];
extern uint8_t g_can2_0x4fe_tx_data[];

extern uint8_t g_can_supercap_tx_Data[];

/* Exported function declarations ---------------------------------------------*/

void can_init(CAN_HandleTypeDef *hcan, CanCallback callback_function);

void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id);

uint8_t can_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);

void can_period_elapsed_callback();

#endif // BSP_CAN_H_

/************************ COPYRIGHT(C) HNUST-DUST **************************/
