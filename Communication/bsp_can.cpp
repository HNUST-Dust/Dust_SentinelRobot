/**
 * @file bsp_can.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CanManageObject g_can1_manage_object = {0};
CanManageObject g_can2_manage_object = {0};

// CAN通信发送缓冲区
uint8_t g_can1_0x1ff_tx_data[8];
uint8_t g_can1_0x200_tx_data[8];
uint8_t g_can1_0x2ff_tx_data[8];
uint8_t g_can1_0x3fe_tx_data[8];
uint8_t g_can1_0x4fe_tx_data[8];

uint8_t g_can2_0x1ff_tx_data[8];
uint8_t g_can2_0x200_tx_data[8];
uint8_t g_can2_0x2ff_tx_data[8];
uint8_t g_can2_0x3fe_tx_data[8];
uint8_t g_can2_0x4fe_tx_data[8];

uint8_t g_can_supercap_tx_data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param callback_function 处理回调函数
 */
void can_init(CAN_HandleTypeDef *hcan, CanCallback callback_function)
{
    if (hcan->Instance == CAN1)
    {
        g_can1_manage_object.can_handler = hcan;
        g_can1_manage_object.callback_function = callback_function;
        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        g_can2_manage_object.can_handler = hcan;
        g_can2_manage_object.callback_function = callback_function;
        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }

    HAL_CAN_Start(hcan);

    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param object_para 编号 | FIFOx | ID类型 | 帧类型
 * @param id ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id)
{
    CAN_FilterTypeDef filter = {0};

    assert_param(hcan != NULL);

    // 解析参数
    uint8_t filter_index = object_para >> 3;                //滤波器索引
    uint8_t fifo_select  = (object_para >> 2) & 0x01;       //FIFO选择
    uint8_t id_type_flag = (object_para >> 1) & 0x01;       //ID类型
    uint8_t frame_type   = object_para & 0x01;              //帧类型


    // 标准ID
    filter.FilterIdHigh = 0x000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = (fifo_select == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    filter.FilterBank = filter_index;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;       //CAN2滤波器为14 - 27

    UNUSED(frame_type);                     // 帧类型在FDCAN中不需要配置

    HAL_CAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t can_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.IDE                   = CAN_ID_STD;               //选择标准ID           
    tx_header.StdId                 = id;                       //配置id
    tx_header.ExtId                 = 0x00;                     //配置0
    tx_header.RTR                   = CAN_RTR_DATA;             //数据帧
    tx_header.DLC                   = length;                   //数据长度
    tx_header.TransmitGlobalTime    = DISABLE;                  //不启用全局时间传输

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &used_mailbox);
}


void can_period_elapsed_callback()
{
    // DJI电机专属
    // CAN1电机

}


/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FIFO_0, 
                    &g_can1_manage_object.rx_buffer.header,
                       g_can1_manage_object.rx_buffer.data);
        g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FIFO_0,
                                &g_can2_manage_object.rx_buffer.header,
                                g_can2_manage_object.rx_buffer.data);
        g_can2_manage_object.callback_function(&g_can2_manage_object.rx_buffer);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1,
                                &g_can1_manage_object.rx_buffer.header,
                                g_can1_manage_object.rx_buffer.data);
        g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1,
                                &g_can2_manage_object.rx_buffer.header,
                                g_can2_manage_object.rx_buffer.data);
        g_can2_manage_object.callback_function(&g_can2_manage_object.rx_buffer);
    }
}


/************************ COPYRIGHT(C) HNUST-DUST **************************/
