#include "dvc_PC_comm.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"

void PcComm::Init()
{
    dwt_init(168);
}

void PcComm::Send_Message()
{
    uint8_t buffer[16];
    memcpy(buffer, &send_data, sizeof(PC_Send_Data));
    usb_transmit(buffer,16);
}

void PcComm::RxCpltCallback()
{
    if (recv_data.Start_Of_Frame == bsp_usb_rx_buffer[0])
    {
        memcpy(recv_data.Yaw, &bsp_usb_rx_buffer[1], 4);
        memcpy(recv_data.Pitch, &bsp_usb_rx_buffer[5], 4);
        recv_data.Fire       = bsp_usb_rx_buffer[9];
        recv_data.CRC16[0]   = bsp_usb_rx_buffer[10];
        recv_data.CRC16[1]   = bsp_usb_rx_buffer[11];
    }
}
