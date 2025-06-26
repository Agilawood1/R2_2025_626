/**
 * @file DL-LN.h
 * @authors 万鹏 7415 (2789152534@qq.com)
 * @authors Huangney
 * @date 2024-11-28 21:56:15
 * @finishdata 2025-1-3 17:39:36
 * @brief 应用库，无线自组网通讯模块，基于DL-LN32P
 * @version 0.2
*/

/** 
 * @details 在v0.2中，更改了部分代码逻辑，新增了一些预定义
*/


#ifndef DL_LN_H
#define DL_LN_H

#define DL_LN_QUERY_PORT 0x80    //  查询请求端口
#define DL_LN_REPLY_PORT 0x81    //  查询回复端口


// #include "stm32f1xx_hal.h"  // 包含 HAL 库中的 UART 相关定义
#include "stm32f4xx_hal.h"
#include "std_msg.h"

/**
 * 
 */
typedef struct DL_LN_Msg
{
    uint8_t frame_head;     // 帧头
    uint8_t frame_tail;     // 帧尾
    uint16_t addr;          // 消息的地址
    uint8_t source_port;    // 消息的源端口
    uint8_t targ_port;      // 消息的目标端口
    uint8_t data[24];     // 数据位开始处
    uint8_t data_length;    // 数据长度
    uint8_t real_length;    // （含转义）数据长度
}DL_LN_Msg;


extern uint8_t DL_LN_rx_buffer[255];  // 接收缓冲区
extern  uint8_t DL_LN_rx_len;
extern uint8_t DL_LN_rx_end_flag;

extern uint8_t PC_tx_buffer[100];    // 转发给上位机的数据缓冲区
extern uint8_t query_data[5];     // 用于广播的数据query 71 75 65 72 79
extern uint8_t reply_data[5];     // 用于接收广播返回的数据reply 71 75 65 72 79

#define SET_ADDRESS_MODE        0x01  // 设置地址模式
#define SET_NETWORK_ID_MODE     0x02  // 设置网络 ID 模式
#define SET_CHANNEL_MODE        0x03  // 设置信道模式
#define SET_BAUD_RATE_MODE      0x04  // 设置波特率模式


/*      DL_LN3x 模块的解码函数      */
int DL_LN_decode(uint8_t recv_msg[], DL_LN_Msg* recv_msg_info);

/*      DL_LN3x 模块的预定义操作函数      */
void DL_LN_reply_node_info(UART_HandleTypeDef *DL_LN_UART, char name[], uint16_t addr);




/*      以下是普通的函数声明      */

// 解析链路质量信息并发送到上位机
void DL_LN_parse_link_quality(UART_HandleTypeDef DL_LN_UART);

// DL_LN 模块发送命令
void DL_LN_send_command(UART_HandleTypeDef *DL_LN_UART, const uint8_t *command,size_t length);

// 读取DL_LN 模块的状态
void DL_LN_read_state(UART_HandleTypeDef *DL_LN_UART,UART_HandleTypeDef *PC_UART);

// 执行链路质量测试
void DL_LN_link_quality_test(UART_HandleTypeDef DL_LN_UART,uint16_t module_1_address, uint16_t module_2_address);  

// 设置DL_LN 模块的模式等参数
void DL_LN_set(UART_HandleTypeDef *DL_LN_UART, UART_HandleTypeDef *PC_UART, uint8_t mode, uint32_t param1, uint8_t param2);

// 重启DL_LN 模块
void DL_LN_restart(UART_HandleTypeDef *DL_LN_UART);

// DL_LN 模块发送数据
void DL_LN_send_packet(UART_HandleTypeDef *DL_LN_UART, uint8_t send_port, uint8_t recv_port, uint16_t addr, uint8_t *data, int data_length);

void DL_LN_send_packet_all_addr(
    UART_HandleTypeDef *DL_LN_UART, uint8_t send_port, 
    uint8_t recv_port, uint8_t *data, int data_length);

// 点亮测试用的LED
void DL_LN_LED_ON(UART_HandleTypeDef *DL_LN_UART);

void DL_LN_LISTEN(UART_HandleTypeDef *DL_LN_UART,UART_HandleTypeDef *PC_UART); 
#endif /* DL_LN_H */
