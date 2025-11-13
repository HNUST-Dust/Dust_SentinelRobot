/**
 * @file bsp_usb.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_USB_H
#define BSP_USB_H

#ifdef __cplusplus
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
extern uint8_t *send_buffer;

/* Exported function declarations --------------------------------------------*/

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */
void usb_init(USBCallback tx_cbk, USBCallback rx_cbk);

void usb_transmit(uint8_t *buffer, uint16_t len); // 通过usb发送数据
void usb_refresh(); // 重新枚举USB设备

#ifdef __cplusplus
}
#endif
#endif



