#ifndef MOTOR_DM_H
#define MOTOR_DM_H
#include "stm32f4xx_hal.h"
#include "can.h"

#define DM_HALFROUND_PERSEC 1.57        // 每秒半圈
#define DM_ROUND_PERSEC 3.14        // 每秒一圈
#define DM_TWOROUND_PERSEC 6.28        // 每秒两圈


#define DM_MOTORID_SHOOTER 0x02
#define DM_MOTORID_MAGAZINE 0x06


// 不同的消息类型，需要不同的ID偏置量
#define DM_MESTYPE_INIT 0x00
#define DM_MESTYPE_POSITION 0x100

void DM_get_posispd_buffer(uint8_t buf[], float posi, float speed);
void DM_get_PosiSpd_bufferlim(uint8_t buf[], float posi, float speed, float lim_H, float lim_L);
void DM_buf_send(uint8_t id, uint8_t bufs[], int id_offset);
void DM_buf_send_can(CAN_HandleTypeDef* hcan, uint8_t id, uint8_t bufs[], int id_offset);

extern uint8_t DM_enable_buf[8];
#endif