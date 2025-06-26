#include "motor_vesc.h"
#include "string.h"
float read_current = 0;
float read_duty = 0;
int read_rpm = 0;
float test_duty_value = 0.1;

// 全局电机实例
MotorVESC motor_vesc_1;
MotorVESC motor_vesc_2;
MotorVESC motor_vesc_3;

/// @name motor_vesc_get_rpm
/// @brief 返回指定标签电机的真实转速
/// @param motor_id ：0-3
int motor_vesc_get_rpm(int motor_id)
{
    if (motor_id == motor_vesc_1.motor_id)
    {
        read_rpm = motor_vesc_1.motor_rpm_real;
        return read_rpm;
    }
    else if (motor_id == motor_vesc_2.motor_id)
    {
        read_rpm = motor_vesc_2.motor_rpm_real;
        return read_rpm;
    }
    else if (motor_id == motor_vesc_3.motor_id)
    {
        read_rpm = motor_vesc_3.motor_rpm_real;
        return read_rpm;
    }
    
    else return 0;
}

/**
 * @name motor_vesc_handle
 * @brief 这个函数处理了收到的信息并回复电机
 * @note 
 * @details 本函数内完成了回复电机（设置电机速度等）的操作；如果有需要的操作可以在此更改
 * @warning 
 */
void motor_vesc_handle(MotorVescRecvData vesc_recvs)
{
    // 解码数据来自于CAN总线上的谁
    uint8_t can_id = vesc_recvs.rx_header.ExtId & 0xff;

    MotorVESC* targ_motor_vesc;

    // 和哪个电机匹配就和谁发
    if (can_id == motor_vesc_1.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_1;
    }
    else if (can_id == motor_vesc_2.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_2;
    }
		    else if (can_id == motor_vesc_3.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_3;
    }

    MotorVESC_SetMotorRPM(targ_motor_vesc, targ_motor_vesc->motor_rpm_set);


    // 获取电调上报的消息类型
    CanPacketType vesc_status_type = (CanPacketType)(vesc_recvs.rx_header.ExtId >> 8);
    // 解码
    switch (vesc_status_type)
    {
        case CAN_PACKET_STATUS: // 第一类上报
        {
            targ_motor_vesc->motor_duty_real = (vesc_recvs.recv_data[6] * 256 + vesc_recvs.recv_data[7]) / 1000.0;

            read_current = (vesc_recvs.recv_data[4] * 256 + vesc_recvs.recv_data[5]) / 10.0;
            
            int32_t temp = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
                | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
                
            if (temp < 50000 && temp > -50000)
            {
                targ_motor_vesc->motor_rpm_real = temp;
            }
            
            // targ_motor_vesc->motor_rpm_real = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
            //     | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
        }
    }
}


// 初始化MotorVESC结构体
void MotorVESC_Init(MotorVESC* motor, CAN_HandleTypeDef* can_n, int motor_id, int motor_can_id)
{
    motor->motor_id = motor_id;
    motor->motor_can_id = motor_can_id;
    motor->targ_can_n = can_n;
    motor->motor_duty_real = 0;
    motor->motor_rpm_real = 0;
    motor->motor_duty_set = 0;
    motor->motor_rpm_set = 0;
}



// 设置电机转速（外部调用）
int motor_vesc_set_rpm(int motor_id, float set_rpm)
{
    if (motor_id == motor_vesc_1.motor_id)
    {
        motor_vesc_1.motor_rpm_set = (int)set_rpm;
        return 1;
    }
    else if (motor_id == motor_vesc_2.motor_id)
    {
        motor_vesc_2.motor_rpm_set = (int)set_rpm;
        return 1;
    }
		    else if (motor_id == motor_vesc_3.motor_id)
    {
        motor_vesc_3.motor_rpm_set = (int)set_rpm;
        return 1;
    }
}



// 设置电机转速
void MotorVESC_SetMotorRPM(MotorVESC* motor, int RPM)
{
    MotorVESC_SendCanTXBuffer(motor, CAN_PACKET_SET_RPM, RPM);
}

// 设置电机占空比
void MotorVESC_SetMotorDuty(MotorVESC* motor, float duty)
{
    MotorVESC_SendCanTXBuffer(motor, CAN_PACKET_SET_DUTY, duty);
}

uint8_t canTx_text[8];
// 发送CAN消息
void MotorVESC_SendCanTXBuffer(MotorVESC* motor, CanPacketType cmd_type, float values)
{
    static uint32_t txmailbox;            // CAN 邮箱
    CAN_TxHeaderTypeDef TxMsg;            // TX 消息

    // 配置标准CAN参数
    TxMsg.StdId = 0x00;    // 低8位为CAN_ID，高21位为指令ID
    TxMsg.ExtId = (cmd_type << 8 | motor->motor_can_id);
    TxMsg.IDE = CAN_ID_EXT;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 8;

    uint8_t txbuf[8] = {0};
    switch (cmd_type)
    {
        case CAN_PACKET_SET_DUTY:
        {
            int32_t data;
            data = (int32_t)(values * 100000) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        case CAN_PACKET_SET_RPM:
        {
            int32_t data;
            data = (int32_t)(values) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        default:
            break;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(motor->targ_can_n) == 0);     // 等待CAN邮箱
    HAL_CAN_AddTxMessage(motor->targ_can_n, &TxMsg, txbuf, &txmailbox);
		memcpy(canTx_text,txbuf,8);
}

float limit_abs(float targ_num, float limit)
{
    if (targ_num > limit)
    {
        return limit;
    }
    if (targ_num < -limit)
    {
        return -limit;
    }
    return targ_num;
}