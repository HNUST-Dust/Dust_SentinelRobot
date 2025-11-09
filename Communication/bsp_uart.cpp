/**
 * @file bsp_usart.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-08
 * 
 * @copyright Copyright (c) 2025
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/

Uart_Instance uart1_instance = {0};
Uart_Instance uart3_instance = {0};
Uart_Instance uart6_instance = {0};

/* Private function declarations ---------------------------------------------*/

/**
 * @brief ：printf重定向函数（不用管）
 * 
 * @param ch 
 * @return int 
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* function prototypes -------------------------------------------------------*/

/**
 * @brief ：UART初始化函数
 * 
 * @param huart ：句柄
 * @param callback_function ：回调函数
 * @param rx_buffer_length ：接收缓冲区长度
 */
void uart_init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
	if(huart->Instance == USART1)
	{
		uart1_instance.uart_handle = huart;
		uart1_instance.callback_function = callback_function;
		uart1_instance.rx_buffer_length = rx_buffer_length;
		HAL_UARTEx_ReceiveToIdle_DMA(uart1_instance.uart_handle, uart1_instance.rx_buffer, uart1_instance.rx_buffer_length);
	}
	if(huart->Instance == USART3)
	{
		uart3_instance.uart_handle = huart;
		uart3_instance.callback_function = callback_function;
		uart3_instance.rx_buffer_length = rx_buffer_length;
		HAL_UARTEx_ReceiveToIdle_DMA(uart3_instance.uart_handle, uart3_instance.rx_buffer, uart3_instance.rx_buffer_length);
	}
	if(huart->Instance == USART6)
	{
		uart6_instance.uart_handle = huart;
		uart6_instance.callback_function = callback_function;
		uart6_instance.rx_buffer_length = rx_buffer_length;
		HAL_UARTEx_ReceiveToIdle_DMA(uart6_instance.uart_handle, uart6_instance.rx_buffer, uart6_instance.rx_buffer_length);
	}
}

/** 
 * @brief ：UART接收DMA空闲中断函数
 * 
 * @param huart ：句柄
 * @param Size ：接收长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		if(uart1_instance.callback_function != NULL)
		{
			uart1_instance.callback_function(uart1_instance.rx_buffer, Size);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(uart1_instance.uart_handle, uart1_instance.rx_buffer, uart1_instance.rx_buffer_length);
	}
	if(huart->Instance == USART3)
	{
		if(uart3_instance.callback_function != NULL)
		{
			uart3_instance.callback_function(uart3_instance.rx_buffer, Size);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(uart3_instance.uart_handle, uart3_instance.rx_buffer, uart3_instance.rx_buffer_length);
	}
	if(huart->Instance == USART6)
	{
		if(uart6_instance.callback_function != NULL)
		{
			uart6_instance.callback_function(uart6_instance.rx_buffer, Size);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(uart6_instance.uart_handle, uart6_instance.rx_buffer, uart6_instance.rx_buffer_length);
	}
}

/**
 * @brief ：UART1回调函数
 * 
 * @param buffer ：接收缓冲区
 * @param length ：接收长度
 */
void uart1_callback_function(uint8_t* buffer, uint16_t length)
{	
	printf("%s\n", buffer);
}

void uart6_callback_function(uint8_t* buffer, uint16_t length)
{	
	
}



