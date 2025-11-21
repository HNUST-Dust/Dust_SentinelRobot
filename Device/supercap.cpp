/**
 * @file supercap.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "supercap.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Supercap初始化函数
 * 
 * @param hcan 
 * @param can_rx_id 
 * @param can_tx_id1 
 * @param can_tx_id2 
 * @param chassis_power_limit 
 * @param chassis_power_buffer 
 * @param discharge_power_limit 
 * @param charge_power_limit 
 * @param switch_status 
 * @param record_status 
 */
void Supercap::Init(CAN_HandleTypeDef *hcan, uint16_t can_rx_id, uint16_t can_tx_id1, uint16_t can_tx_id2, 
                    uint16_t chassis_power_limit, uint16_t chassis_power_buffer, int16_t discharge_power_limit, 
                    uint16_t charge_power_limit, SupercapSwitchStatus switch_status, SupercapRecordStatus record_status)
{
    if (hcan->Instance == CAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == CAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    can_rx_id_ = can_rx_id;
    can_tx_id1_ = can_tx_id1;
    can_tx_id2_ = can_tx_id2;

    // 设置底盘功率限制
    chassis_power_limit_ = chassis_power_limit;
    // 设置底盘功率缓冲
    chassis_power_buffer_ = chassis_power_buffer;
    // 放电功率限制
    discharge_power_limit_ = discharge_power_limit;
    // 充电功率限制
    charge_power_limit_ = chassis_power_limit;

    // 设置超电状态
    supercap_control_.control.supercap_switch = switch_status;
    supercap_control_.control.supercap_record = record_status;

    static const osThreadAttr_t kSupercapTaskAttr = {
        .name = "supercap_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Supercap::TaskEntry, this, &kSupercapTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Supercap::TaskEntry(void *argument)
{
    Supercap *self = static_cast<Supercap *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief SupercapCAN通讯接收回调函数
 * 
 * @param rx_data 
 */
void Supercap::CanRxCpltCallback(uint8_t *rx_data)
{
    // 滑动窗口，判断超级电容在线状态
    flag_ += 1;
    DataProcess();
}

/**
 * @brief Supercap在线回调函数
 * 
 */
void Supercap::AlivePeriodElapsedCallback()
{
    // TODO:待实现
}

/**
 * @brief Supercap数据处理函数
 * 
 */
void Supercap::DataProcess()
{
    uint8_t* temp_buffer = can_manage_object_->rx_buffer.data;

    int16_t v_code = (int16_t)(temp_buffer[0] << 8 | temp_buffer[1]);
    int16_t i_code = (int16_t)(temp_buffer[2] << 8 | temp_buffer[3]);
    uint16_t all  = (uint16_t)(temp_buffer[4] << 8 | temp_buffer[5]);

    received_data_.supercap_voltage = int16_to_float(v_code, 32000, -32000, 30, 0);
    received_data_.supercap_current = int16_to_float(i_code, 32000, -32000, 20, -20);
    received_data_.supercap_status_code.all = all;
    chassis_power_ = - received_data_.supercap_voltage * received_data_.supercap_current;

    // printf("%f,%f,%f\n", received_data_.supercap_voltage, -received_data_.supercap_current, chassis_power_);
}

/**
 * @brief Supercap发送回调函数
 * 
 */
void Supercap::SendPeriodElapsedCallback()
{
    uint8_t temp_buffer[8];
    temp_buffer[0] =  chassis_power_buffer_ >> 8;
    temp_buffer[1] = (uint8_t)chassis_power_buffer_;
    temp_buffer[2] = 0;
    temp_buffer[3] = 0;
    temp_buffer[4] = 0;
    temp_buffer[5] = 0;
    temp_buffer[6] = 0;
    temp_buffer[7] = 0;

    can_send_data(can_manage_object_->can_handler, can_tx_id1_, temp_buffer, 8);

    temp_buffer[0] = chassis_power_limit_ >> 8;
    temp_buffer[1] = (uint8_t)chassis_power_limit_;
    temp_buffer[2] = discharge_power_limit_ >> 8;
    temp_buffer[3] = (uint8_t)discharge_power_limit_;
    temp_buffer[4] = charge_power_limit_ >> 8;
    temp_buffer[5] = (uint8_t)charge_power_limit_;
    temp_buffer[6] = supercap_control_.all >> 8;
    temp_buffer[7] = (uint8_t)supercap_control_.all;
    
    can_send_data(can_manage_object_->can_handler, can_tx_id2_, temp_buffer, 8);
}

/**
 * @brief Supercap任务函数
 * 
 */
void Supercap::Task()
{
    for (;;)
    {
        AlivePeriodElapsedCallback();
        SendPeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(10));
    }
}


/************************ COPYRIGHT(C) HNUST-DUST **************************/
