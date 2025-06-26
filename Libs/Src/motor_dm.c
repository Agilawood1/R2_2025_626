#include "motor_dm.h"

// 达妙用数据,使能
uint8_t DM_enable_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

/**
 * @name 获得位置速度数组
 * @details 输入位置和速度（均为RAD），获得一个可以发送给达妙电机的数组
 */
void DM_get_posispd_buffer(uint8_t buf[], float posi, float speed)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&posi;
	vbuf=(uint8_t*)&speed;

    buf[0] = *pbuf;
    buf[1] = *(pbuf+1);
    buf[2] = *(pbuf+2);
    buf[3] = *(pbuf+3);
    
    buf[4] = *vbuf;
    buf[5] = *(vbuf+1);
    buf[6] = *(vbuf+2);
    buf[7] = *(vbuf+3);
}

/**
 * @name 获得位置速度数组
 * @details 输入位置和速度（均为RAD），获得一个可以发送给达妙电机的数组
 */
void DM_get_PosiSpd_bufferlim(uint8_t buf[], float posi, float speed, float lim_H, float lim_L)
{
    if (posi > lim_H)
    {
      posi = lim_H;
    }
    if (posi < lim_L)
    {
      posi = lim_L;
    }
    
    
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&posi;
	  vbuf=(uint8_t*)&speed;


    buf[0] = *pbuf;
    buf[1] = *(pbuf+1);
    buf[2] = *(pbuf+2);
    buf[3] = *(pbuf+3);
    
    buf[4] = *vbuf;
    buf[5] = *(vbuf+1);
    buf[6] = *(vbuf+2);
    buf[7] = *(vbuf+3);
}


// 定义反馈帧解析函数
void parseFeedbackFrame(uint8_t frame[8]) {
    // 解析ID
    uint8_t ID = frame[0] & 0xFF;

    // 解析故障信息
    uint8_t ERR = (frame[0] >> 4) & 0x0F;
    const char* errorMessage = "";
    switch (ERR) {
        case 8: errorMessage = "超压"; break;
        case 9: errorMessage = "欠压"; break;
        case 0xA: errorMessage = "过电流"; break;
        case 0xB: errorMessage = "MOS过温"; break;
        case 0xC: errorMessage = "电机线圈过温"; break;
        case 0xD: errorMessage = "通讯丢失"; break;
        case 0xE: errorMessage = "过载"; break;
        default: errorMessage = "无故障"; break;
    }

    // 解析位置信息
    uint16_t POS = (frame[1] << 8) | frame[2];

    // 解析速度信息
    uint16_t VEL = (frame[3] << 4) | (frame[4] >> 4);

    // 解析扭矩信息
    uint16_t T = (frame[4] & 0x0F) << 8 | frame[5];

    // 解析MOS温度
    uint8_t T_MOS = frame[6];

    // 解析电机线圈温度
    uint8_t T_Rotor = frame[7];

    // 打印解析结果
    //printf("控制器ID: %d\n", ID);
    //printf("故障信息: %s\n", errorMessage);
    //printf("电机位置: %d\n", POS);
    //printf("电机速度: %d\n", VEL);
    //printf("电机扭矩: %d\n", T);
    //printf("驱动上MOS的平均温度: %d℃\n", T_MOS);
    //printf("电机内部线圈的平均温度: %d℃\n", T_Rotor);
}


/**
 * @name 给达妙发送消息
 * @details 默认CAN2
 */
void DM_buf_send(uint8_t id, uint8_t bufs[], int id_offset)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = (uint8_t)8 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_STD ;		          // 发送的帧是扩展帧
  TxMsg.StdId = id_offset + id;             // CAN_ID
  TxMsg.TransmitGlobalTime = DISABLE; 

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);     // 等待CAN邮箱
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, bufs, &txmailbox);

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);     // 等待CAN邮箱
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, bufs, &txmailbox);
}

/**
 * @name 给达妙发送消息
 * @details 默认CAN2
 */
void DM_buf_send_can(CAN_HandleTypeDef* hcan, uint8_t id, uint8_t bufs[], int id_offset)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = (uint8_t)8 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_STD ;		          // 发送的帧是扩展帧
  TxMsg.StdId = id_offset + id;             // CAN_ID
  TxMsg.TransmitGlobalTime = DISABLE; 

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);     // 等待CAN邮箱

	HAL_CAN_AddTxMessage(hcan, &TxMsg, bufs, &txmailbox);
}