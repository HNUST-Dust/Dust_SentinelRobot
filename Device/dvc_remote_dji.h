/**
 * @file dvc_remote_dji.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DVC_REMOTE_DJI_H__
#define __DVC_REMOTE_DJI_H__
/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

enum RemoteDjiAliveStatus
{
    REMOTE_DJI_STATUS_DISABLE = 0,
    REMOTE_DJI_STATUS_ENABLE  = 1,
};

/**
 * @brief 遥控按键状态
 * 
 */
enum RemoteDjiSwitchStatus
{
    SWITCH_UP    = (uint8_t)1,
    SWITCH_MID   = (uint8_t)3,
    SWITCH_DOWN  = (uint8_t)2,
};

/**
 * @brief 遥控原始数据
 * 
 */
struct RemoteDjiData
{
    uint16_t ch0, ch1, ch2, ch3;
    uint8_t s1, s2;
};

/**
 * @brief 遥控输出
 * 
 */
struct RemoteDjiOutput
{
    uint8_t switch_l, switch_r;
    float chassis_x, chassis_y;      // x, y, r 采用右手系
    float rotation;
    float pitch;       
};

/**
 * @brief DjiDR16遥控器
 * 
 */
class RemoteDjiDR16
{
public:
    // 遥控器输出数据
    RemoteDjiOutput output_;

    // 遥控器状态
    RemoteDjiAliveStatus remote_dji_alive_status = REMOTE_DJI_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    inline float GetLeftX();

    inline float GetLeftY();

    inline float GetRightX();

    inline float GetRightY();

    inline uint8_t GetSwitchL();

    inline uint8_t GetSwitchR();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

protected:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 原始数据
    RemoteDjiData data_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    // 归一化线性转换参数

    float k_nor = 1.0f / 660.0f;

    float c_nor = -256.0f / 165.0f;

    // pitch线性转换参数（当前为-27度为摇杆最低，-23为摇杆最高）

    float k_pitch = 1.f / 330.f;

    float c_pitch = -4637.f / 165.f;

    // 掉线清理数据函数

    void ClearData();

    // 内部数据处理函数

    void DataProcess(uint8_t* buffer);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取左摇杆X值
 * 
 * @return float 
 */
inline float RemoteDjiDR16::GetLeftX()
{
    return output_.rotation;
}

inline float RemoteDjiDR16::GetLeftY()
{
    return output_.pitch;
}

inline float RemoteDjiDR16::GetRightX()
{
    return output_.chassis_x;
}

inline float RemoteDjiDR16::GetRightY()
{
    return output_.chassis_y;
}

inline uint8_t RemoteDjiDR16::GetSwitchL()
{
    return output_.switch_l;
}

inline uint8_t RemoteDjiDR16::GetSwitchR()
{
    return output_.switch_r;
}

#endif