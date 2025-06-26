/*
 * @Author: scy
 * @Date: 2024-02-07 17:31:48
 * @LastEditors: scy
 * @LastEditTime: 2024-02-25 11:42:59
 * @FilePath: \MDK-ARMf:\Intelligent Car\competition item\Robocon\sterring wheel\BSP\Inc\bsp_fdcan.h
 * @Description: 
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"

void bspcan_filter_init_recv_all(CAN_HandleTypeDef *_hcan);
uint8_t bspcan_C620_send_msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t len, uint8_t more);
#endif 
