/*
 * @Author: Agilawood

 * @file: 全场定位封装库(FULL-Scope Positioning Module Library)
 * 
 * @LastEditors: Agilawood
 * 
 * @LastEditTime: 2024-11-18
 * 
 * @FilePath: E:\up70_2025\Action\MDK-ARM\Action.uvprojx-μVision
 * 
 * @Note: 1.本文件采用UTF-8编码
 *        2.action使用RS232物理层协议，需使用TTL转RS232模块( 型号: Freestrong TTL to RS485/RS232 Rev 1.0 )
 *        3.硬件接法说明(Crucial)：模块的 T 接“TTL转RS232模块”的 A/TXD, R 接 B/RXD ; MCU的串口TX 接 “TTL转RS232模块” 的 TX， RX 接 RX；模块VIN接输出稳定的5V，并做好“共地”处理
 *        4.串口波特率为115200 bit/s
 *        5.该库使用的extern全局变量： Odometer odometer_data;//里程计数据
 *
 * @brief: 该库支持接收和更新x、y、yaw三方向坐标，并通过dma接受机制减少CPU负担和提高数据传输效率
 * 
 * @API: 1.更新坐标函数：update_odom(char a, float value);
 *       2.dma中断、接收初始化函数：void odom_dma_init();
 *       3.dma中断处理函数：void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
 *       4.里程计数据接口函数：Odometer get_odometer_data();
 *
 * @example：一、必要准备
 * 
 *           1.在主函数中调用以下函数初始化dma接收： 
 *             odom_dma_init();
 *           
 *           2.在interrupt.c中的对应串口中断函数中【如 void USART1_IRQHandler(void) 】，加入以下函数：
 *             USER_UART_IRQHandler(&ACTION_USART_HANDLE);
 * 
 *          二、使用示例
 * 
 *           1.可以根据需要使用以下函数更新坐标： 
 *             update_odom('x', value); // (x坐标)
 *             update_odom('y', value); // (y坐标)
 *             update_odom('j', value); // (yaw角) 
 * 
 *           2.可供用户读取 x，y，yaw，yaw_speed 四个值
 *             方法：定义一个Odometer型的变量，调用 get_odometer_data() 接收数据
 * 
 * @contact：Feel free to contact me at 1979716201@qq.com for any question                
 *                
 * Copyright (c) 2024 by Agilawood, All Rights Reserved.
 */


#include "action_OPS.h"

static uint8_t dma_rx_buffer[30];//存储action发的数据包，30个字节
static Odometer odometer_data;//里程计数据

/* 字符串拼接函数 */
void stractString(char str1[], char str2[], int num)
{
	int i = 0, j = 0;
    while(str1[i] != '\0')
    {
        i++;
    }
    for(j = 0; j < num; j++)
    {
        str1[i++] = str2[j];
    }
}

 /* 以下为给Action发数据的函数 */

/**
  *@brief  更新X坐标函数，与update_Odometer()配合使用
  *@param  需要更新的float型 x坐标，该模块规定必须为float
  *@retval void 
  *@plus   使用共用体主要是为了方便数据类型的转换(float和char)，此外还可以节省内存
  */  

static void update_x(float new_x)
{
    char x[8] = "ACTX";
    static union 
    {
        float X;
        char data[4];
    }NewSet;
    
    NewSet.X = new_x;

    stractString(x, NewSet.data, 4);    //整合数据
    
    for(int i = 0; i < 8; i++)    //串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&x[i], 1, 1000);//内部自置等待上一笔数据发送完的flag
    }

    HAL_Delay(10);//应使用vTaskDelay
}

/* 更新标定Y坐标 */
static void update_y(float new_y)
{
    char y[8] = "ACTY";
    static union 
    {
        float Y;
        char data[4];
    }NewSet;
    
    NewSet.Y = new_y;

    stractString(y, NewSet.data, 4);    //整合数据

    for(int i = 0; i < 8; i++)    //串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&y[i], 1, 1000);//内部自置等待上一笔数据发送完的flag
    }

    HAL_Delay(10);//应使用vTaskDelay
}

/* 更新标定yaw角 */
static void update_yaw(float new_yaw)
{
    char yaw[8] = "ACTJ";//char型存疑
    static union 
    {
        float J;
        char data[4];
    }NewSet;
    
    NewSet.J = new_yaw;

    stractString(yaw, NewSet.data, 4);//整合数据
    
    for(int i = 0; i < 8; i++)//串口把数据发送给action
    {
        HAL_UART_Transmit(&ACTION_USART_HANDLE, (uint8_t*)&yaw[i], 1, 1000);//内部自置等待上一笔数据发送完的flag，是否应该转换为uint8_t存疑
    }

    HAL_Delay(10);//应使用vTaskDelay
}

/**
  * @brief 更新三方向坐标 
  * @note  更新yaw角时，传的参数是j; value必须为float型 
  * @param char x,y,j
  * @param float value
  * @retval void
  */
void update_odom(char a, float value)
{
    if(a == 'x')
    {
        update_x(value);
    }
    if(a == 'y')
    {
        update_y(value);
    }
    if(a == 'j')
    {
        update_yaw(value);
    }
}

/**************************************************************************************************************************************************************************************/

/* 以下为接收位姿信息的函数 */

/**
  * @brief dma中断初始化
  * @param void
  * @retval void
  */
void odom_dma_init()
{
	__HAL_UART_ENABLE_IT(&ACTION_USART_HANDLE, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&ACTION_USART_HANDLE, (uint8_t*)dma_rx_buffer, 30);
}

/**
  * @brief 更新三方向坐标 
  * @note  更新yaw角时，传的参数是j; value必须为float型 
  * @param UART_HandleTypeDef *huart，dma使用的串口句柄
  * @param Odometer *des，数据的导入终点
  * @param uint8_t *src，通过dma获得的action全场定位发来的数据源
  * @retval {*}
  */
void uartIdleCallback(UART_HandleTypeDef *huart, Odometer *des, uint8_t *src)
{
	
    HAL_UART_DMAStop(&ACTION_USART_HANDLE); // 停止本次DMA传输 
                                                       
    uint8_t data_length  = 30 - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 计算接收到的数据长度，getcounter中算的是dma流中剩的字节数   
                   
    memcpy(des, src, data_length);
	
//    memset(receive_buff,0,data_length);// 清零接收缓冲区
	
    data_length = 0;
    
    HAL_UART_Receive_DMA(&ACTION_USART_HANDLE, (uint8_t*)src, 30); // 重启开始DMA传输 每次30字节数据                    
}

/**
  * @brief 串口dma中断处理函数 
  * @param UART_HandleTypeDef *huart，dma使用的串口句柄
  * @retval odometer的数据
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{	
    if(huart->Instance == ACTION_USART_HANDLE.Instance)                                   
    {	
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // 判断是否是空闲中断   
        {	 
            __HAL_UART_CLEAR_IDLEFLAG(huart); // 清除空闲中断标志（否则会一直不断进入中断）                   
            
            uartIdleCallback(huart, &odometer_data, dma_rx_buffer); // 调用中断回调函数                       
        }
    }
}

/**
  * @brief 里程计数据接口函数 
  * @param void
  * @retval odometer的数据
  */
Odometer get_odometer_data()
{
	return odometer_data;
}


/**
  * @brief 里程计将结构体转为三维向量（其中Yaw角将由Deg变为Rad）（其中x，y单位由 毫米 变为 米 ）
  * @param Odometer odom_sturct
  * @retval 向量
  */
Vec3 odom_get_tf_vec(Odometer odom_sturct)
{
    Vec3 tf_vec;
    tf_vec.x = odom_sturct.x / 1000.0;
    tf_vec.y = odom_sturct.y / 1000.0;
    tf_vec.z = odom_sturct.yaw * 3.1415926 / 180.0;
    
    return tf_vec;
}




