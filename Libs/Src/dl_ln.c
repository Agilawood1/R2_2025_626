/**
 * @file dl_ln.h
 * @author 万鹏 7415 (2789152534@qq.com)
 * @date 2024-11-28 21:56:15
 * @finishdata 2025-1-3 17:39:36
 * @brief 应用库，无线自组网通讯模块，基于DL-LN32P
 * @version 0.1
 * @note
 */

// #include "stm32f1xx.h"
// #include "stm32f1xx_hal_uart.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "string.h"
#include "stdio.h"
// #include "stdint.h"
#include "dl_ln.h"

/**
 * @brief 看我！别忘了配置！！！
 */

// 端口定义 尽可能不要冲突
/*外部端口*/
#define READ_PORT 0X90
#define LINK_TEST_PORT_local 0X85
#define SET_PORT 0X95

/*内部端口*/
#define LED_PORT 0X20
#define LOCAL_PORT 0X21
#define LINK_TEST_PORT_remote 0X23

// 特殊信息定义
#define LOCAL_ADDRESS 0X00
#define msg_length_to_PC 20    // 向上位机发送的数据长度，不要太大浪费
#define MAX_PACKET_SIZE 255    // 定义最大包长
#define msg_length_to_send 255 // 向其他模块发送的数据长度

#define TEMP 0xAA     // 占位符，存在的意义是填充占位，等待更替
#define CMD_TEMP 0xAA // 占位符，存在的意义是填充命令位
#define ADR_TEMP 0XAA // 占位符，存在的意义是填充地址位

// 设置-命令定义
#define SET_IP_CMD 0x11
#define SET_ID_CMD 0x12
#define SET_CHANNEL_CMD 0x13
#define SET_BAUD_RATE_CMD 0x14
#define RESTART_CMD 0x10

// 设置-模式定义
#define SET_ADDRESS_MODE 0x01
#define SET_NETWORK_ID_MODE 0x02
#define SET_CHANNEL_MODE 0x03
#define SET_BAUD_RATE_MODE 0x04

// 宏定义返回包长度
#define REC_IP_LEN 10
#define REC_ID_LEN 10
#define REC_CH_LEN 9
#define REC_BPS_LEN 9
#define REC_LEN_OK 8 // 这个是包代表修改过程中的正确修改

// 数据接收缓冲区
uint8_t DL_LN_rx_buffer[255]; // 接收缓冲区
uint8_t DL_LN_rx_len;
uint8_t PC_tx_buffer[100]; // 转发给上位机的数据缓冲区

uint8_t query_data[5] = {0x71, 0x75, 0x65, 0x72, 0x79};     // ascii 'query'
uint8_t reply_data[5] = {0x72, 0x65, 0x70, 0x6c, 0x79};     // ascii 'reply'

static int read_state = 0;
static int send_state = 0;
uint8_t DL_LN_rx_end_flag;

// 读取命令定义
const uint8_t LED_ON[] = {0xFE, 0X05, 0X80, 0X20, 0X00, 0X00, 0X0A, 0XFF};
const uint8_t READ_IP_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x01, 0xFF};
const uint8_t READ_ID_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x02, 0xFF};
const uint8_t READ_CHANNEL_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x03, 0xFF};
const uint8_t READ_BAUD_RATE_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x04, 0xFF};
// 设置命令定义
uint8_t SET_CMD_IP_ID[] = {0xFE, 0x07, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, CMD_TEMP, TEMP, TEMP, 0xFF};
uint8_t SET_CMD_CH_BPS[] = {0xFE, 0x06, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, CMD_TEMP, TEMP, 0xFF};
// 链路测试命令定义
uint8_t LINK_QUALITY_TEST[] = {0xFE, 0x06, LINK_TEST_PORT_local, LINK_TEST_PORT_remote, ADR_TEMP, ADR_TEMP, ADR_TEMP, ADR_TEMP, 0xFF};
// 重启命令定义
uint8_t RESTART[] = {0xFE, 0x05, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, RESTART_CMD, 0xFF};
// 普通发送信息
uint8_t SEND_MSG[msg_length_to_send];

/***************************************DL-LN信息读取相关函数****************************************************/
/**
 * @brief 将波特率代码转换为实际波特率，就是波特率解码，参见手册
 * @param baud_code 波特率代码
 * @return 实际波特率
 * @note 表函数，不引出，外部没用
 */
uint32_t DL_LN_decode_baud_rate(uint8_t baud_code)
{
    switch (baud_code)
    {
    case 0x00:
        return 2400;
    case 0x01:
        return 4800;
    case 0x02:
        return 9600;
    case 0x03:
        return 14400;
    case 0x04:
        return 19200;
    case 0x05:
        return 28800;
    case 0x06:
        return 38400;
    case 0x07:
        return 57600;
    case 0x08:
        return 115200;
    case 0x09:
        return 230400;
    case 0x0A:
        return 125000;
    case 0x0B:
        return 250000;
    case 0x0C:
        return 500000;
    default:
        return 0; // 未知波特率，一般不会出现
    }
}

typedef enum
{
    IP = 1, // IP 值为 1
    ID = 2, // ID 值为 2
    CH = 3, // CH 值为 3
    BPS = 4 // BPS 值为 4
} DL_LN_Enum;

/**
 * @brief 将实际波特率转换为波特率代码，就是波特率编码
 * @param baud_code 实际波特率
 * @return 波特率代码
 * @note 表函数，不引出，外部没用
 */
uint8_t DL_LN_encode_baud_rate(uint32_t baud_rate)
{
    switch (baud_rate)
    {
    case 2400:
        return 0x00;
    case 4800:
        return 0x01;
    case 9600:
        return 0x02;
    case 14400:
        return 0x03;
    case 19200:
        return 0x04;
    case 28800:
        return 0x05;
    case 38400:
        return 0x06;
    case 57600:
        return 0x07;
    case 115200:
        return 0x08;
    case 230400:
        return 0x09;
    case 125000:
        return 0x0A;
    case 250000:
        return 0x0B;
    case 500000:
        return 0x0C;
    default:
        return 0xFF; // 未知波特率，表示错误
    }
}

/**
 * @brief 解析模块信息并根据当前状态依次读取和发送相关数据。
 *
 * 该函数根据`read_state`的值逐步解析模块的波特率、模块地址、网络ID和频道信息。
 * 每读取一个数据后，更新`read_state`并发送相应的读取命令来获取下一个信息。
 */
/**
 * @brief 解析DL-LN模块的接收到的数据
 * @param buffer     接收到的数据缓冲区
 * @param buffer_len 接收到的数据长度
 * @param DL_LN_UART UART句柄，用于发送反馈信息
 * 不引出，外部没用
 */
void DL_LN_parse_module_info(uint8_t *buffer, uint16_t buffer_len, UART_HandleTypeDef *DL_LN_UART)
{
    // 防止数据溢出
    if (buffer_len < 9)
    {
        //        char error_message[] = "Error: Buffer too short.\r\n";
        //        HAL_UART_Transmit(DL_LN_UART, (uint8_t*)error_message, strlen(error_message), 10);
        return;
    }

    // 解析状态机，控制解析逻辑
    switch (read_state)
    {
    case 0:
    {
        // 解析模块地址
        uint16_t module_address = buffer[7] | (buffer[8] << 8);
        char address_message[50];
        sprintf(address_message, "Module Address: 0x%04X\r\n", module_address);
        HAL_UART_Transmit(DL_LN_UART, (uint8_t *)address_message, strlen(address_message), 10);
        read_state++; // 切换到读取网络ID的状态
        break;
    }
    case 1:
    {
        // 解析网络ID
        uint16_t network_id = buffer[7] | (buffer[8] << 8);
        char network_id_message[50];
        sprintf(network_id_message, "Network ID: 0x%04X\r\n", network_id);
        HAL_UART_Transmit(DL_LN_UART, (uint8_t *)network_id_message, strlen(network_id_message), 10);
        read_state++; // 切换到读取频道的状态
        break;
    }
    case 2:
    {
        // 解析频道
        uint8_t channel = buffer[7];
        char channel_message[50];
        sprintf(channel_message, "Channel: 0x%02X\r\n", channel);
        HAL_UART_Transmit(DL_LN_UART, (uint8_t *)channel_message, strlen(channel_message), 10);
        read_state++; // 切换到读取波特率的状态
        break;
    }
    case 3:
    {
        // 解析波特率
        uint8_t baud_code = buffer[7];
        uint32_t baud_rate = DL_LN_decode_baud_rate(baud_code);
        char baud_rate_message[50];
        if (baud_rate != 0)
        {
            sprintf(baud_rate_message, "Baud Rate: %lu bps\r\n", baud_rate);
        }
        else
        {
            sprintf(baud_rate_message, "Baud Rate: Unknown Code 0x%02X\r\n", baud_code);
        }
        HAL_UART_Transmit(DL_LN_UART, (uint8_t *)baud_rate_message, strlen(baud_rate_message), 10);
        read_state = 0; // 状态重置
        break;
    }
    default:
        read_state = 0; // 避免进入未定义的状态
        break;
    }
}

/**
 * @brief 发送命令到UART接口，使用异步传输。
 *
 * 该函数通过UART接口异步发送给定的命令。它使用`HAL_UART_Transmit_IT`函数来传输命令数据，确保命令在后台被发送。
 *
 * @param command 发送的命令数据
 * @param length 命令数据的长度
 */
void DL_LN_send_command(UART_HandleTypeDef *DL_LN_UART, const uint8_t *command, size_t length)
{
    HAL_UART_Transmit(DL_LN_UART, command, length, HAL_MAX_DELAY);
}

/**
 * @brief 心跳检测，检测DL-LN是不是活着
 **/
void DL_LN_LED_ON(UART_HandleTypeDef *DL_LN_UART)
{
    DL_LN_send_command(DL_LN_UART, LED_ON, sizeof(LED_ON));
}

/**
 * @brief 发送读取模块信息的命令并启动数据接收。
 *
 * 该函数依次发送读取模块地址、网络ID、频道和波特率的命令，并启动UART的接收中断来接收返回的数据。
 * 每发送一个命令后，都会通过`HAL_UART_Receive_IT`启动接收，确保能够及时接收数据。
 *
 * @note 该函数中的`HAL_Delay`用来在发送每个命令后等待一段时间，以确保模块能够处理命令。
 */
void DL_LN_read(UART_HandleTypeDef *DL_LN_UART)
{
    switch (send_state)
    {
    case 0:
    {
        DL_LN_send_command(DL_LN_UART, READ_IP_CMD, sizeof(READ_IP_CMD));
        send_state++;
        break;
    }
    case 1:
    {
        DL_LN_send_command(DL_LN_UART, READ_ID_CMD, sizeof(READ_ID_CMD));
        send_state++;
        break;
    }
    case 2:
    {
        DL_LN_send_command(DL_LN_UART, READ_CHANNEL_CMD, sizeof(READ_CHANNEL_CMD));
        send_state++;
        break;
    }
    case 3:
    {
        DL_LN_send_command(DL_LN_UART, READ_BAUD_RATE_CMD, sizeof(READ_BAUD_RATE_CMD));
        send_state++;
        break;
    }
    default:
        send_state = 0;
        break;
    }
}

void DL_LN_USART_APP(UART_HandleTypeDef *DL_LN_UART, UART_HandleTypeDef *PC_UART, int mode)
{
    if (DL_LN_rx_end_flag == 1)
    {
        // 拷贝数据到本地缓冲区，确保不会被清除
        uint8_t local_buffer[255];
        memcpy(local_buffer, DL_LN_rx_buffer, DL_LN_rx_len); // 复制全局缓冲区到本地变量
        switch (mode)
        {
        case 1: // 解析数据，使用局部变量而不是全局变量
            DL_LN_parse_module_info(local_buffer, DL_LN_rx_len, PC_UART);
            break;
        case 2:
            HAL_UART_Transmit(PC_UART, local_buffer, DL_LN_rx_len, 10);
        }

        // 清除全局缓冲区
        memset(DL_LN_rx_buffer, 0, sizeof(DL_LN_rx_buffer));

        // 重置全局变量
        DL_LN_rx_len = 0;
        DL_LN_rx_end_flag = 0;

        // 重新启动DMA接收
        HAL_UART_Receive_DMA(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));
    }
}
/**
 * @brief 一键读取函数
 *
 * @param DL_LN_UART 链接DL_LN的端口
 * @param PC_UART 链接上位机的端口
 */
void DL_LN_read_state(UART_HandleTypeDef *DL_LN_UART, UART_HandleTypeDef *PC_UART)
{
    int mode = 1;
    for (int i = 0; i < 5; i++)
    {
        DL_LN_USART_APP(DL_LN_UART, PC_UART, mode);
        DL_LN_read(DL_LN_UART);
        HAL_Delay(10);
    }
}

void DL_LN_LISTEN(UART_HandleTypeDef *DL_LN_UART, UART_HandleTypeDef *PC_UART)
{
    DL_LN_USART_APP(DL_LN_UART, PC_UART, 2);
}
/***************************************DL-LN链路质量测试相关函数****************************************************/

/**
 * @brief 解析链路质量信息（RSSI），并将其发送到上位机。
 *
 * 该函数从接收到的字节流中提取RSSI数据并将其转换为链路质量值。如果接收到的数据为`0x80`，则认为没有数据,存储为0
 * 然后将链路质量信息格式化为字符串，并通过UART异步发送给上位机。
 */
void DL_LN_parse_link_quality(UART_HandleTypeDef DL_LN_UART)
{
    uint8_t rssi_data = DL_LN_rx_buffer[7];                            // 接收到的信号强度指示（RSSI）数据
    int8_t link_quality = (rssi_data != 0x80) ? (int8_t)rssi_data : 0; // 0x80 表示无数据

    // 格式化链路质量信息并发送到上位机
    char message[msg_length_to_PC];
    int len = sprintf(message, "Link Quality: %d\n", link_quality);
    HAL_UART_Transmit_IT(&DL_LN_UART, (uint8_t *)message, len); // 异步发送链路质量信息
}

/**
 * @brief UART接收完成中断回调函数，根据接收到的数据类型进行相应处理。
 *
 * 该函数会在每次UART接收完成时触发。它首先检查接收到的数据包头，然后根据不同的命令类型解析数据。
 * 如果是链路质量测试数据包，则调用`DL_LN_parse_link_quality`进行解析；如果是模块信息读取数据包，则调用`DL_LN_parse_module_info`进行解析。
 * 在处理完成后，函数继续启动UART接收。
 *
 * @param huart 指向UART处理结构体的指针
 */

/**
 * @brief 发送链路质量测试命令，测试模块之间的链路质量。
 *
 * 该函数将两个模块的地址（以小端格式）写入链路质量测试命令数据包，并通过UART接口发送该数据包。
 * 数据包的发送使用`HAL_UART_Transmit_IT`异步传输方式，确保链路质量测试命令被成功传输。
 *
 * @param module_1_address 第一个模块的地址
 * @param module_2_address 第二个模块的地址
 */
void DL_LN_link_quality_test(UART_HandleTypeDef DL_LN_UART, uint16_t module_1_address, uint16_t module_2_address)
{
    // 发送模块B的地址（小端格式）
    LINK_QUALITY_TEST[4] = module_1_address & 0xFF;        // 小端格式：低字节
    LINK_QUALITY_TEST[5] = (module_1_address >> 8) & 0xFF; // 小端格式：高字节

    // 发送模块C的地址（小端格式）
    LINK_QUALITY_TEST[6] = module_2_address & 0xFF;        // 小端格式：低字节
    LINK_QUALITY_TEST[7] = (module_2_address >> 8) & 0xFF; // 小端格式：高字节

    // 发送数据包
    HAL_UART_Transmit_IT(&DL_LN_UART, PC_tx_buffer, sizeof(PC_tx_buffer));
}

/***************************************DL-LN信息设置相关函数****************************************************/

/**
 * @brief 设置 DL-LN 模块的各种参数。
 *
 * @param mode 配置模式，指定要设置的参数类型。
 *             - IP: 设置模块地址
 *             - ID: 设置网络 ID
 *             - CH: 设置信道
 *             - BPS: 设置波特率
 * @param param1 参数 1 如果为两字节数据，这里是第一位，如果是一字节，那就一位
 *
 * @param param2 参数 2 如果为两字节数据，这里是第二位，如果是一字节，这里随便配置，反正没用
 *
 * 示例DL_LN_set(huart1,IP,0X01,0X02);	DL_LN_set(huart1,BPS,115200,0X00);
 */
void DL_LN_set(UART_HandleTypeDef *DL_LN_UART, UART_HandleTypeDef *PC_UART, uint8_t mode, uint32_t param1, uint8_t param2)
{
    uint8_t bps_encoded = DL_LN_encode_baud_rate(param1);

    switch (mode)
    {
    case IP:                              // 设置地址
        SET_CMD_IP_ID[6] = SET_IP_CMD;    // 命令字
        SET_CMD_IP_ID[7] = param2 & 0xFF; // 地址低字节
        SET_CMD_IP_ID[8] = param1 & 0xFF; // 地址高字节
        DL_LN_send_command(DL_LN_UART, SET_CMD_IP_ID, sizeof(SET_CMD_IP_ID));

        HAL_Delay(10);
        DL_LN_read_state(DL_LN_UART, PC_UART);
        break;

    case ID:                              // 设置网络ID
        SET_CMD_IP_ID[6] = SET_ID_CMD;    // 命令字
        SET_CMD_IP_ID[7] = param2 & 0xFF; // 网络ID低字节
        SET_CMD_IP_ID[8] = param1 & 0xFF; // 网络ID高字节
        DL_LN_send_command(DL_LN_UART, SET_CMD_IP_ID, sizeof(SET_CMD_IP_ID));

        HAL_Delay(10);
        DL_LN_read_state(DL_LN_UART, PC_UART);
        break;

    case CH:                                 // 设置信道
        SET_CMD_CH_BPS[6] = SET_CHANNEL_CMD; // 命令字
        SET_CMD_CH_BPS[7] = param1;          // 新信道值
        DL_LN_send_command(DL_LN_UART, SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));

        HAL_Delay(10);
        DL_LN_read_state(DL_LN_UART, PC_UART);
        break;

    case BPS: // 设置波特率
        SET_CMD_CH_BPS[6] = SET_BAUD_RATE_CMD; // 命令字
        SET_CMD_CH_BPS[7] = bps_encoded;       // 新波特率值
        DL_LN_send_command(DL_LN_UART, SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));

        HAL_Delay(10);
        DL_LN_read_state(DL_LN_UART, PC_UART);
        break;

    default:
        // 如果传入了无效的 mode，什么也不做
        break;
    }
}

/**
 * 和上面的函数一样，只不过不会上报，用法一样。
 **/
void DL_LN_set_NOPC(UART_HandleTypeDef *DL_LN_UART, uint8_t mode, uint32_t param1, uint8_t param2)
{
    uint8_t bps_encoded = DL_LN_encode_baud_rate(param1);

    switch (mode)
    {
    case IP:                              // 设置地址
        SET_CMD_IP_ID[6] = SET_IP_CMD;    // 命令字
        SET_CMD_IP_ID[7] = param2 & 0xFF; // 地址低字节
        SET_CMD_IP_ID[8] = param1 & 0xFF; // 地址高字节
        DL_LN_send_command(DL_LN_UART, SET_CMD_IP_ID, sizeof(SET_CMD_IP_ID));

        HAL_Delay(10);

        break;

    case ID:                              // 设置网络ID
        SET_CMD_IP_ID[6] = SET_ID_CMD;    // 命令字
        SET_CMD_IP_ID[7] = param2 & 0xFF; // 网络ID低字节
        SET_CMD_IP_ID[8] = param1 & 0xFF; // 网络ID高字节
        DL_LN_send_command(DL_LN_UART, SET_CMD_IP_ID, sizeof(SET_CMD_IP_ID));

        HAL_Delay(10);

        break;

    case CH:                                 // 设置信道
        SET_CMD_CH_BPS[6] = SET_CHANNEL_CMD; // 命令字
        SET_CMD_CH_BPS[7] = param1;          // 新信道值
        DL_LN_send_command(DL_LN_UART, SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));

        HAL_Delay(10);

        break;

    case BPS: // 设置波特率
        SET_CMD_CH_BPS[6] = SET_BAUD_RATE_CMD; // 命令字
        SET_CMD_CH_BPS[7] = bps_encoded;       // 新波特率值
        DL_LN_send_command(DL_LN_UART, SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));

        HAL_Delay(10);

        break;

    default:
        // 如果传入了无效的 mode，什么也不做
        break;
    }
}

/**
 * @brief 发送重启命令以使模块生效最新设置。
 */
void DL_LN_restart(UART_HandleTypeDef *DL_LN_UART)
{
    DL_LN_send_command(DL_LN_UART, RESTART, sizeof(RESTART));
}

/***************************************DL-LN信息设置相关函数****************************************************/

/**
 * @brief 封装数据包并发送
 *
 * @param send_port 发送端口
 * @param recv_port 接收端口
 * @param target_address 目标地址（大端模式输入）例如0x1234
 * @param data 包内容指针
 */
void DL_LN_send_packet(UART_HandleTypeDef *DL_LN_UART, uint8_t send_port, uint8_t recv_port, uint16_t addr, uint8_t *data, int data_length)
{
    memset(SEND_MSG, 0, MAX_PACKET_SIZE);

    // memcpy(&SEND_MSG[6], data, data_length);

    // 数据内容长度受转义字符影响，同时 粘贴数据内容
    uint8_t escape_len = 0;
    for (int i = 0; i < data_length; i++)
    {
        if (data[i] == 0xff)     // 执行转义ff
        {
            SEND_MSG[6 + i + escape_len] = 0xfe;
            escape_len++;
            SEND_MSG[6 + i + escape_len] = 0xfd;
            data_length++;
        }
        else if (data[i] == 0xfe)     // 执行转义fe
        {
            SEND_MSG[6 + i + escape_len] = 0xfe;
            escape_len++;
            SEND_MSG[6 + i + escape_len] = 0xfc;
            data_length++;
        }
        else
            SEND_MSG[6 + i + escape_len] = data[i];
    }
    

    // 包头
    SEND_MSG[0] = 0xFE;

    // 数据长度（自动计算从第3位到包尾前一位的字节数）
    uint8_t packet_length = 1 + 1 + 2 + data_length - escape_len; // 发送端口(1) + 接收端口(1) + 目标地址(2) + 数据长度（不含转义）
    SEND_MSG[1] = packet_length;

    // 发送端口
    SEND_MSG[2] = send_port;

    // 接收端口
    SEND_MSG[3] = recv_port;

    // 目标地址（小端模式存储）
    SEND_MSG[4] = addr & 0xFF; // 低字节
    SEND_MSG[5] = addr >> 8; // 高字节

    // 包尾
    SEND_MSG[6 + data_length] = 0xFF;

    // 通过 UART 发送数据包
    HAL_UART_Transmit_DMA(DL_LN_UART, SEND_MSG, 7 + data_length);
}


void DL_LN_send_packet_all_addr(
    UART_HandleTypeDef *DL_LN_UART, uint8_t send_port, 
    uint8_t recv_port, uint8_t *data, int data_length)
{
    memset(SEND_MSG, 0, MAX_PACKET_SIZE);

    // 数据内容长度

    // 包头
    SEND_MSG[0] = 0xFE;

    // 数据长度（自动计算从第3位到包尾前一位的字节数）
    uint8_t packet_length = 1 + 1 + 2 + data_length; // 发送端口(1) + 接收端口(1) + 目标地址(2) + 数据长度
    SEND_MSG[1] = packet_length;

    // 发送端口
    SEND_MSG[2] = send_port;

    // 接收端口
    SEND_MSG[3] = recv_port;

    // 目标地址（小端模式存储）,需要转义
    SEND_MSG[4] = 0xFE; // 低字节
    SEND_MSG[5] = 0xFD; // 低字节

    SEND_MSG[6] = 0xFE; // 高字节
    SEND_MSG[7] = 0xFD; // 高字节

    // 数据内容
    memcpy(&SEND_MSG[8], data, data_length);

    // 包尾
    SEND_MSG[8 + data_length] = 0xFF;

    // 通过 UART 发送数据包
    HAL_UART_Transmit_DMA(DL_LN_UART, SEND_MSG, 9 + data_length);
}


int DL_LN_decode(uint8_t recv_msg[], DL_LN_Msg* recv_msg_info)
{
    // 获取帧信息
    recv_msg_info->frame_head = recv_msg[0];
    recv_msg_info->data_length = recv_msg[1] - 4;

    // 从未转义的理论结束位开始，检验是否传输完成
    recv_msg_info->real_length = recv_msg_info->data_length + 7;
    while (1)
    {
        if (recv_msg[recv_msg_info->real_length - 1] == 0xff)     // 检查到帧尾了
        {
            recv_msg_info->frame_tail = recv_msg[recv_msg_info->real_length];
            break;
        }
        else if (recv_msg_info->real_length > 64)
        {
            return 0;       // 传输失败
        }
        
        
        recv_msg_info->real_length++;
    }

    // 接下来解码数据
    uint8_t escape_len = 0;
    for (int i = 0; i + escape_len < recv_msg_info->real_length - 7; i++)
    {
        if (recv_msg[6 + i + escape_len] != 0xfe)        // 没发现转义，直接放进去
        {
            recv_msg_info->data[i] = recv_msg[6 + i + escape_len];
        }
        else
        {
            if (recv_msg[7 + i + escape_len] == 0xfd)
            {
                recv_msg_info->data[i] = 0xff;
                escape_len++;
            }
            else if (recv_msg[7 + i + escape_len] == 0xfc)
            {
                recv_msg_info->data[i] = 0xfe;
                escape_len++;
            }
        }
    }
    
    // 收录地址、端口信息
    recv_msg_info->addr = ((uint16_t)recv_msg[5] * 256 | recv_msg[4]);
    recv_msg_info->source_port = recv_msg[2];
    recv_msg_info->targ_port = recv_msg[3];
    
    return 1;   // 传输成功
}


/**
 * @name 回复节点信息
 */
void DL_LN_reply_node_info(UART_HandleTypeDef *DL_LN_UART, char name[], uint16_t addr)
{
    DL_LN_send_packet(DL_LN_UART, DL_LN_REPLY_PORT, DL_LN_QUERY_PORT, addr, name, 8);
}