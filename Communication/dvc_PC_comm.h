/**
 * @file dvc_PC_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PC_COMM_H
#define PC_COMM_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 自瞄发送结构体
 * 
 */
struct Send_AutoAim_Data
{
    uint8_t start_of_frame = 0x5A;
    uint8_t armor = 0x00;
    uint8_t end_of_frame[6] = {0xCD, 0xCC, 0x00, 0x00, 0x00, 0x00};
    float yaw = 0;
    float pitch = 0;
};

/**
 * @brief 自瞄接收结构体
 * 
 */
struct Recv_AutoAim_Data
{
    uint8_t start_of_frame = 0xA5;
    uint8_t yaw[4] = {0};
    uint8_t pitch[4] = {0};
    uint8_t fire = 0x00;
    uint8_t crc16[2] = {0};
};

/**
 * @brief 导航接收结构体
 * 
 */
struct Recv_Navigation_Data
{
    uint8_t start_of_frame = 0x6A;
    uint8_t linear_x[4] = {0};
    uint8_t linear_y[4] = {0};
    uint8_t crc16[2] = {0};
};

/**
 * @brief Pc命令类
 * 
 */
class PcComm
{
public:
    // 发送自瞄数据
    Send_AutoAim_Data send_autoaim_data = 
    {
        0x5A,
        0x00,
        {0xCD, 0xCC, 0x00, 0x00, 0x00, 0x00},
        0,
        0,
    };
    // 接收自瞄数据
    Recv_AutoAim_Data recv_autoaim_data = 
    {
        0xA5,
        {0},
        {0},
        0X00,
        {0},
    };
    // 接收导航数据
    Recv_Navigation_Data recv_navigation_data = 
    {
        0x6A,
        {0},
        {0},
        {0},
    };

    void Init();

    void Send_Message();

    void RxCpltCallback();
private:

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
