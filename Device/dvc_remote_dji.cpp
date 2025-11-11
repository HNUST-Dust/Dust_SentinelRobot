/**
 * @file dvc_remote_dji.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_dji.h"
#include "stdio.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief DjiDR16遥控初始化函数
 * 
 * @param huart uart句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接受缓冲区长度
 */
void RemoteDjiDR16::Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
    uart_manage_object_->uart_handle = huart;
    uart_manage_object_->callback_function = callback_function;
    uart_manage_object_->rx_buffer_length = rx_buffer_length;
    uart_init(huart, callback_function, rx_buffer_length);

    static const osThreadAttr_t kRemoteTaskAttr = {
        .name = "kRemoteTaskAttr",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(RemoteDjiDR16::TaskEntry, this, &kRemoteTaskAttr);
}

/**
 * @brief DjiDR16遥控
 * 
 * @param argument 
 */
void RemoteDjiDR16::TaskEntry(void *argument)
{
    RemoteDjiDR16 *self = static_cast<RemoteDjiDR16 *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief DjiDR16数据处理函数
 * 
 */
void RemoteDjiDR16::DataProcess(uint8_t* buffer)
{
    data_.ch0 =  ((int16_t)buffer[0]        | ((int16_t)buffer[1] << 8)) & 0x07FF;
    data_.ch1 = (((int16_t)buffer[1] >> 3)  | ((int16_t)buffer[2] << 5)) & 0x07FF;
    data_.ch2 = (((int16_t)buffer[2] >> 6)  | ((int16_t)buffer[3] << 2)  | ((int16_t)buffer[4] << 10)) &  0x07FF;
    data_.ch3 = (((int16_t)buffer[4] >> 1)  | ((int16_t)buffer[5] << 7)) & 0x07FF;

    data_.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    data_.s2 = ((buffer[5] >> 4) & 0x0003);

    // 上板数据
    output_.pitch = k_pitch * data_.ch3 + c_pitch;
    
    // 下板数据
    output_.chassis_x  = data_.ch0;
    output_.chassis_y  = data_.ch1;
    output_.rotation   = data_.ch2;

    // 通用数据
    output_.switch_l = data_.s1;
    output_.switch_r = data_.s2;
}

/**
 * @brief DjiDR16转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    // 读取数据值
    DataProcess(buffer);
}

/**
 * @brief DjiDR16清理数据函数
 * 
 */
void RemoteDjiDR16::ClearData()
{
    output_.pitch = k_pitch * 1024 + c_pitch;
    output_.chassis_x = output_.chassis_y = output_.rotation = 1024;
    output_.switch_l = output_.switch_r = 3;
}

/**
 * @brief DjiDR16检测在线回调函数
 * 
 */
void RemoteDjiDR16::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_dji_alive_status = REMOTE_DJI_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_dji_alive_status = REMOTE_DJI_STATUS_ENABLE;
    }
    pre_flag_ = flag_;
}

/**
 * @brief DjiDR16任务函数
 * 
 */
void RemoteDjiDR16::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(500));
    }
}

