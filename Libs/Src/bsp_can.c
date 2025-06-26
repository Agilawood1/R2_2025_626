/*
 * @Author: scy
 * @Date: 2024-02-07 17:31:48
 * @LastEditors: scy
 * @LastEditTime: 2024-02-25 11:42:35
 * @FilePath: \MDK-ARMf:\Intelligent Car\competition item\Robocon\sterring wheel\BSP\Src\bsp_fdcan.c
 * @Description:
 */
#include "bsp_can.h"

uint8_t turnback_msg[4] = {0};

/**
 * @description: 接收过滤器（全部接收）
 * @param {FDCAN_HandleTypeDef} *_hfdcan使用的FDCAN
 * @return {*}无
 */
void bspcan_filter_init_recv_all(CAN_HandleTypeDef *can_n)
{

    CAN_FilterTypeDef FilterConfig;

    /* 配置CAN过滤器 */
    FilterConfig.FilterBank = 0;                      // 使用过滤器组0
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 屏蔽位模式
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位宽过滤器
    // 期望的ID设为0，掩码设置高11位全检查，但需调整以实现范围过滤
    FilterConfig.FilterIdHigh = 0x0000;               // 期望的ID高16位（标准ID左移5位）
    FilterConfig.FilterIdLow = 0x0000;                // 低16位未使用
    // 设置掩码：仅允许ID高11位中最高有效位未设置的ID（即ID < 0x800）
    // 具体掩码计算：ID < 100 (0x64)，需允许ID高7位为0，故掩码高7位设为1以检查这些位
    FilterConfig.FilterMaskIdHigh = 0x0000; // 高7位掩码（0x7F <<5 = 0x3F80）
    FilterConfig.FilterMaskIdLow = 0x0000;            // 低16位掩码未使用
    FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0; // 绑定到FIFO0

    
    
    FilterConfig.FilterActivation = ENABLE;           // 启用过滤器
		HAL_CAN_ConfigFilter(can_n, &FilterConfig);
//    if (HAL_CAN_ConfigFilter(can_n, &FilterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
		if (can_n == &hcan2) {
         FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO1; // 绑定到FIFO1
        FilterConfig.FilterBank = 14; 
			HAL_CAN_ActivateNotification(can_n, CAN_IT_RX_FIFO1_MSG_PENDING);
    } else if(can_n == &hcan1){
        FilterConfig.FilterBank = 0; 
			HAL_CAN_ActivateNotification(can_n, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    //HAL_CAN_ActivateNotification(can_n, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @description: 发送信息（普通CAN格式）
 * @param {CAN_HandleTypeDef} *hcan 使用的CAN
 * @param {uint8_t} *msg 发送数据的指针
 * @param {uint32_t} len 发送数据的长度（最大8字节）
 * @param {int} motor_id 电机ID
 * @return {*} 1：发送成功 0：发送错误
 */
uint8_t bspcan_C620_send_msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t len, uint8_t more)
{
    CAN_TxHeaderTypeDef CanTxHeader;
    uint32_t txMailbox; // 用于存储发送邮箱号
    
    // 配置标准ID
    if (more) {
        CanTxHeader.StdId = 0x1FF;  // 11位标准ID
    } else {
        CanTxHeader.StdId = 0x200;  // 11位标准ID
    }
    
    // 配置报文头
    CanTxHeader.IDE = CAN_ID_STD;   // 标准帧
    CanTxHeader.RTR = CAN_RTR_DATA;      // 数据帧
    CanTxHeader.DLC = len;               // 数据长度（0-8）
    CanTxHeader.TransmitGlobalTime = DISABLE; // 禁用全局时间传输
    
    // 调用HAL库发送函数
    if (HAL_CAN_AddTxMessage(hcan, &CanTxHeader, msg, &txMailbox) != HAL_OK) {
        return 0;
    }
    return 1;
}
