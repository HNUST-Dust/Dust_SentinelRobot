/**
 * @file dvc_PC_comm.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_PC_comm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief PcComm初始化函数
 * 
 */
void PcComm::Init()
{
    dwt_init(168);
}

/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::Send_Message()
{
    uint8_t buffer[16];  // 明确的16字节
    
    buffer[0] = send_autoaim_data.start_of_frame;
    buffer[1] = send_autoaim_data.armor;
    memcpy(&buffer[2], send_autoaim_data.end_of_frame, 6);
    memcpy(&buffer[8], &send_autoaim_data.yaw, 4);
    memcpy(&buffer[12], &send_autoaim_data.pitch, 4);
    
    usb_transmit(buffer, 16);
}

/**
 * @brief PcComm接收回调函数
 * 
 */
void PcComm::RxCpltCallback()
{
    if (bsp_usb_rx_buffer[0] == 0xFF)
    {
        union {float f; uint8_t b[4] ;} conv;

        conv.b[0] = bsp_usb_rx_buffer[1];
        conv.b[1] = bsp_usb_rx_buffer[2];
        conv.b[2] = bsp_usb_rx_buffer[3];
        conv.b[3] = bsp_usb_rx_buffer[4];

        send_autoaim_data.pitch = conv.f;
    }
    
    if (bsp_usb_rx_buffer[0] == 0xFE)
    {
        union {float f; uint8_t b[4] ;} conv;

        conv.b[0] = bsp_usb_rx_buffer[1];
        conv.b[1] = bsp_usb_rx_buffer[2];
        conv.b[2] = bsp_usb_rx_buffer[3];
        conv.b[3] = bsp_usb_rx_buffer[4];

        send_autoaim_data.yaw = conv.f;
    }


    if (bsp_usb_rx_buffer[0] == recv_autoaim_data.start_of_frame)
    {
        memcpy(recv_autoaim_data.yaw,   &bsp_usb_rx_buffer[1], 4);
        memcpy(recv_autoaim_data.pitch, &bsp_usb_rx_buffer[5], 4);

        recv_autoaim_data.fire      = bsp_usb_rx_buffer[9];
        recv_autoaim_data.crc16[0]  = bsp_usb_rx_buffer[10];
        recv_autoaim_data.crc16[1]  = bsp_usb_rx_buffer[11];
    }
    else if(bsp_usb_rx_buffer[0] == recv_navigation_data.start_of_frame)
    {
        memcpy(recv_navigation_data.linear_x, &bsp_usb_rx_buffer[1], 4);
        memcpy(recv_navigation_data.linear_y, &bsp_usb_rx_buffer[5], 4);

        recv_navigation_data.crc16[0] = bsp_usb_rx_buffer[9];
        recv_navigation_data.crc16[1] = bsp_usb_rx_buffer[10];
    }
}
