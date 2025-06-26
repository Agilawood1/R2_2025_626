#ifndef __ACTION_OPS_H__
#define __ACTION_OPS_H__

#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "std_msg.h"

#define ACTION_USART_HANDLE huart6

#pragma pack(2)//设置为2字节对齐，否则在memcpy时会出错
typedef struct Odometer
{
    //Note:按照action发送的数据包，结构体成员的顺序必须如下
    uint16_t frame_head;//帧头0D0A
    float yaw;//逆时针旋转为角度正方向，范围0~180，-180~0
    float pith;
    float roll;
    float x;
    float y;
    float yaw_speed;
    uint16_t frame_tail;//帧尾0A0D
}Odometer;//数据包一共28字节
#pragma pack()

//extern uint8_t dma_rx_buffer[30];//存储action发的数据包，30个字节
//extern Odometer odometer_data;//里程计数据

void update_odom(char a, float value);
void odom_dma_init();
Vec3 odom_get_tf_vec(Odometer odom_sturct);

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
Odometer get_odometer_data();

#endif


