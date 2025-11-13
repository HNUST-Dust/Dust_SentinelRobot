/**
 * @file bsp_usb.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

/* Private variables ---------------------------------------------------------*/

uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
uint8_t *send_buffer;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况
void usb_init(USBCallback tx_cbk, USBCallback rx_cbk)
{
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(tx_cbk, rx_cbk); // 获取接收数据指针
}

void usb_transmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len); // 发送
}

void usb_refresh()
{
    // 重新枚举usb设备
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    dwt_delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    dwt_delay(0.1);
}
