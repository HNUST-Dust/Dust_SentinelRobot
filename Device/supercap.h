/**
 * @file supercap.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef DEVICE_SUPERCAP_H_
#define DEVICE_SUPERCAP_H_

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "alg_math.h"
#include "cmsis_os2.h"
#include "user_lib.h"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Supercap开关状态枚举
 * 
 */
enum SupercapSwitchStatus
{
    SUPERCAP_STATUS_SWITCH_DISABLE = 0,
    SUPERCAP_STATUS_SWITCH_ENABLE = 1,
};

/**
 * @brief Supercap记录状态枚举
 * 
 */
enum SupercapRecordStatus
{
    SUPERCAP_STATUS_RECORD_DISABLE = 0,
    SUPERCAP_STATUS_RECORD_ENABLE = 1,
};

/**
 * @brief Supercap状态码联合体
 * 
 */
union SupercapStatusCode
{
    uint16_t all;
    struct
    {
        uint16_t warning : 1;               //报警
        uint16_t cap_v_over : 1;            //电容过压
        uint16_t cap_i_over : 1;            //电容过流
        uint16_t cap_v_low : 1;             //电容欠压
        uint16_t bat_v_low : 1;             //裁判系统欠压
        uint16_t can_receive_miss : 1;      //未读到CAN通信数据
    } status;
};

/**
 * @brief Supercap控制联合体
 * 
 */
union SupercapControl
{
    uint16_t all;
    struct
    {
        uint16_t supercap_switch : 1;    //电容开关
        uint16_t supercap_record : 1;    //记录功能开关
    } control;
};

/**
 * @brief Supercap接收数据结构体
 * 
 */
struct SupercapRecivedData
{
    SupercapStatusCode supercap_status_code;
    float supercap_voltage;
    float supercap_current;
};

/**
 * @brief Supercap发送数据结构体
 * 
 */
struct SupercapSendData
{
    uint16_t chassis_power_buffer;      //底盘功率缓冲
    uint16_t chassis_power_limit;       //机器人底盘功率限制上限
    int16_t discharge_power_limit;      //电容放电功率限制
    uint16_t charge_power_limit;        //电容充电功率限制
    SupercapControl supercap_control;
};

/**
 * @brief Supercap类
 * 
 */
class Supercap
{
public:
    // 底盘功率
    float chassis_power_ = 0.0f;

    void Init(CAN_HandleTypeDef *hcan, uint16_t can_rx_id = 0x030, uint16_t can_tx_id1 = 0x02E, uint16_t can_tx_id2 = 0x02F,
              uint16_t chassis_power_limit = 55, uint16_t chassis_power_buffer = 50, int16_t discharge_power_limit = 50, 
              uint16_t charge_power_limit = 50, SupercapSwitchStatus switch_status = SUPERCAP_STATUS_SWITCH_ENABLE, 
              SupercapRecordStatus record_status = SUPERCAP_STATUS_RECORD_DISABLE);

    void Task();

    void CanRxCpltCallback(uint8_t *rx_data);

    void AlivePeriodElapsedCallback();

    void SendPeriodElapsedCallback();

    inline void SetChassisPowerBuffer(uint16_t chassis_power_buffer);

    inline void SetChassisPowerLimit(uint16_t chassis_power_limit);

    inline void SetDischargePowerLimit(int16_t discharge_power_limit);

    inline void SetChargePowerLimit(uint16_t charge_power_limit);

    inline void SetSupercapCharge(SupercapSwitchStatus supercap_switch_status);

    inline void SetSupercapRecord(SupercapRecordStatus supercap_record_status);

    inline float GetSupercapVoltage();

    inline float GetSupercapCurrent();

    inline uint16_t GetSupercapStatus();

protected:
    // CAN管理模块
    CanManageObject *can_manage_object_ = nullptr;

    // CAN接收id
    uint16_t can_rx_id_ = 0x030;

    // CAN发送id
    uint16_t can_tx_id1_ = 0x02E;
    uint16_t can_tx_id2_ = 0x02F;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    // 超电接收数据
    SupercapRecivedData received_data_;

    // 超电发送数据（未用）
    // SupercapSendData send_data_;

    // 底盘功率缓冲
    uint16_t chassis_power_buffer_;

    // 机器人底盘功率限制上限
    uint16_t chassis_power_limit_;

    // 超电放电功率限制
    int16_t discharge_power_limit_;

    // 超电充电功率限制
    uint16_t charge_power_limit_;     
    
    // 超电控制
    SupercapControl supercap_control_;

    void DataProcess();

    void Output();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief Supercap设置底盘功率缓冲函数
 * 
 * @param chassis_power_buffer 底盘功率缓冲值
 */
inline void Supercap::SetChassisPowerBuffer(uint16_t chassis_power_buffer)
{
    chassis_power_buffer_ = chassis_power_buffer;
}

/**
 * @brief Supercap设置底盘功率限制函数
 * 
 * @param chassis_power_limit 底盘功率限制值
 */
inline void Supercap::SetChassisPowerLimit(uint16_t chassis_power_limit)
{
    chassis_power_limit_ = chassis_power_limit;
}

/**
 * @brief Supercap设置放电功率限制函数
 * 
 * @param discharge_power_limit 放电功率限制值
 */
inline void Supercap::SetDischargePowerLimit(int16_t discharge_power_limit)
{
    discharge_power_limit_ = discharge_power_limit;
}

/**
 * @brief Supercap设置充电功率限制函数
 * 
 * @param charge_power_limit 充电功率限制值
 */
inline void Supercap::SetChargePowerLimit(uint16_t charge_power_limit)
{
    charge_power_limit_ = charge_power_limit;
}

/**
 * @brief Supercap设置开关状态函数
 * 
 * @param supercap_switch_status 开关状态值
 */
inline void Supercap::SetSupercapCharge(SupercapSwitchStatus supercap_switch_status)
{
    supercap_control_.control.supercap_switch = supercap_switch_status;
}

/**
 * @brief Supercap设置记录状态函数
 * 
 * @param supercap_record_status 记录状态值
 */
inline void Supercap::SetSupercapRecord(SupercapRecordStatus supercap_record_status)
{
    supercap_control_.control.supercap_record = supercap_record_status;
}

/**
 * @brief Supercap获取电压函数
 * 
 * @return float 电压值
 */
inline float Supercap::GetSupercapVoltage()
{
    return (received_data_.supercap_voltage);
}

/**
 * @brief Supercap获取电流函数
 * 
 * @return float 电流值
 */
inline float Supercap::GetSupercapCurrent()
{
    return (received_data_.supercap_current);
}

/**
 * @brief Supercap获取状态函数
 * 
 * @return uint16_t 状态位
 */
inline uint16_t Supercap::GetSupercapStatus()
{
    return (received_data_.supercap_status_code.all);
}


#endif // !DEVICE_SUPERCAP_H_
