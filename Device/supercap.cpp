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
#include "bsp_can.h"
#include "cmsis_os.h"

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
 * @param can_tx_id 
 */
void Supercap::Init(CAN_HandleTypeDef *hcan, uint16_t can_rx_id, uint16_t can_tx_id1, uint16_t can_tx_id2)
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

    power_limit_max_ = 55;
    power_compensate_max_ = 50;
    supercap_enable_status_ = SUPERCAP_STATUS_ENABLE;

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
    SupercapRecivedData *temp_buffer = (SupercapRecivedData *)can_manage_object_->rx_buffer.data;

    recived_data_.supercap_work_status = temp_buffer->supercap_work_status;
    recived_data_.supercap_status_code = temp_buffer->supercap_status_code;
    recived_data_.supercap_energy_percent = temp_buffer->supercap_energy_percent;
    recived_data_.chassis_compensate_power = temp_buffer->chassis_compensate_power;
    recived_data_.battery_voltage = temp_buffer->battery_voltage;
}

/**
 * @brief Supercap发送回调函数
 * 
 */
void Supercap::SendPeriodElapsedCallback()
{
    uint8_t temp_buffer[8];
    temp_buffer[0] = 0;
    temp_buffer[1] = 0;
    temp_buffer[2] = 0;
    temp_buffer[3] = 0;
    temp_buffer[4] = 0;
    temp_buffer[5] = 0;
    temp_buffer[6] = 0;
    temp_buffer[7] = 0;
    can_send_data(can_manage_object_->can_handler, can_tx_id1_, temp_buffer, 8);

    temp_buffer[0] = 0;
    temp_buffer[1] = 0;
    temp_buffer[2] = 0;
    temp_buffer[3] = 0;
    temp_buffer[4] = 0;
    temp_buffer[5] = 0;
    temp_buffer[6] = 0;
    temp_buffer[7] = 0;
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
