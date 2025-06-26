#include "rmt_stick.h"
#include "string.h"
#include "logic.h"

RmtJsOriginInfo My_js_origins;
RmtJoystickInfo My_joystick;

uint16_t l_stick_x_origin = 2048;
uint16_t l_stick_y_origin = 2048;
uint16_t r_stick_x_origin = 2048;
uint16_t r_stick_y_origin = 2048;

int wrong = 0;

void encode_joystick_msg(RmtJoystickInfo *js_info, uint8_t *encode_result_msg)
{
    // js_info->left_stick_x_float = js_info->left_stick_x / 2048.0;
    encode_result_msg[0] = 'J'; // 开始标识符

    encode_result_msg[1] = js_info->msg_id_mark >> 8;
    encode_result_msg[2] = js_info->msg_id_mark & 0xff;

    encode_result_msg[3] = js_info->left_stick_x >> 8;
    encode_result_msg[4] = js_info->left_stick_x & 0xff;
    encode_result_msg[5] = js_info->left_stick_y >> 8;
    encode_result_msg[6] = js_info->left_stick_y & 0xff;

    encode_result_msg[7] = js_info->right_stick_x >> 8;
    encode_result_msg[8] = js_info->right_stick_x & 0xff;
    encode_result_msg[9] = js_info->right_stick_y >> 8;
    encode_result_msg[10] = js_info->right_stick_y & 0xff;

    encode_result_msg[MSG_LENGTH - 1] = 'S'; // 结束标识符

    js_info->msg_id_mark++;
}

void encode_reply_msg(uint8_t *msg_recved, uint8_t *encode_result_msg)
{
    encode_result_msg[0] = msg_recved[1];
    encode_result_msg[1] = msg_recved[2];
}

int rmt_check_rmts_status(char status_code[], RMTS_STATUS targ_status)
{
    switch (targ_status)
    {
        case RMTS_CONTROL:
        {
            if (status_code[0] == '1' && status_code[1] == '0' && status_code[2] == '1')
            {
                return 1;
            }
            else return 0;
        }
        break;
        case RMTS_NORMAL:
        {
            if (status_code[0] == '2' && status_code[1] == '1' && status_code[2] == '2')
            {
                return 1;
            }
            else return 0;
        }
        break;
    }
    return 0;
}

int rmt_recv_complete_msg(uint8_t *msg_recv)
{
    if (msg_recv[MSG_LENGTH - 1] == 'S')
    {
        return 1;
    }
    else
        return 0;
}

void rmt_decode_joystick_orimsg(RmtJsOriginInfo *js_info, uint8_t msg_to_decode[], uint8_t id)
{
    switch (id)
    {
        case 0:
        {
            js_info->left_stick_x_low = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 1:
        {
            js_info->left_stick_x_high = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 2:
        {
            js_info->left_stick_y_low = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 3:
        {
            js_info->left_stick_y_high = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }

/********************************************************************************************** */

        case 4:
        {
            js_info->right_stick_x_low = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 5:
        {
            js_info->right_stick_x_high = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 6:
        {
            js_info->right_stick_y_low = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 7:
        {
            js_info->right_stick_y_high = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }

/********************************************************************************************** */

        case 8:
        {
            js_info->left_stick_x_mid = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 9:
        {
            js_info->left_stick_y_mid = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 10:
        {
            js_info->right_stick_x_mid = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }
        case 11:
        {
            js_info->right_stick_y_mid = msg_to_decode[2] * 256 + msg_to_decode[3];
            break;
        }

    default:
        break;
    }
}

/**
 * @brief 用于转换byte 与 实际按钮的 bool 的函数
 */
void rmt_convertBytesToBits(uint8_t Keys_byte_0, uint8_t Keys_byte_1, uint8_t Keys_byte_2, uint8_t bits[24]) {
    // 将字节拆分为位并存储到 decoded_keys 数组中
    for (int i = 0; i < 8; i++) {
        bits[i] = (Keys_byte_0 >> i) & 1;
    }
    for (int i = 0; i < 8; i++) {
        bits[i + 8] = (Keys_byte_1 >> i) & 1;
    }
    for (int i = 0; i < 8; i++) {
        bits[i + 16] = (Keys_byte_2 >> i) & 1;
    }
}

// int rmt_decode_joystick_msg(RmtJoystickInfo *js_info, uint8_t msg_to_decode[])
// {
//     // 验证校验位 和 模式位
//     if (msg_to_decode[9] == ((msg_to_decode[1] + msg_to_decode[3] + msg_to_decode[5] + msg_to_decode[7]) & 0xff) && 
//         msg_to_decode[10] == ((msg_to_decode[0] + msg_to_decode[2] + msg_to_decode[4] + msg_to_decode[6]) & 0xff) &&
//         msg_to_decode[11] == 0x01)
//     {
//         js_info->left_stick_x = (int16_t)(msg_to_decode[0] << 8 | msg_to_decode[1]);
//         js_info->left_stick_y = (int16_t)(msg_to_decode[2] << 8 | msg_to_decode[3]);
//         js_info->right_stick_x = (int16_t)(msg_to_decode[4] << 8 | msg_to_decode[5]);
//         js_info->right_stick_y = (int16_t)(msg_to_decode[6] << 8 | msg_to_decode[7]);
//         return 1;
//     }
//     else
//     {
//         wrong++;
//         return 0;
//     }
// }
int rmt_decode_joystick_msg(RmtJoystickInfo *js_info, uint8_t msg_to_decode[])
{
    // 验证标识符
    if (msg_to_decode[12] == 0x01) {
        // 验证校验位
        uint8_t Keys_byte_0 = msg_to_decode[8];
        uint8_t Keys_byte_1 = msg_to_decode[9];
        uint8_t Keys_byte_2 = msg_to_decode[10];
        uint8_t checksum = msg_to_decode[11];
        if (checksum == (Keys_byte_0 ^ Keys_byte_1 ^ Keys_byte_2)) {
            // 解析摇杆数据
            js_info->left_stick_x = (int16_t)((msg_to_decode[0] << 8) | msg_to_decode[1]);
            js_info->left_stick_y = (int16_t)((msg_to_decode[2] << 8) | msg_to_decode[3]);
            js_info->right_stick_x = (int16_t)((msg_to_decode[4] << 8) | msg_to_decode[5]);
            js_info->right_stick_y = (int16_t)((msg_to_decode[6] << 8) | msg_to_decode[7]);
            rmt_convertBytesToBits(Keys_byte_0, Keys_byte_1, Keys_byte_2, js_info->Keys);
            // 可以在这里解析按钮数据
            // 例如，可以将按钮数据存储到结构体的某个数组中
            return 1;
        }
    }
    // 解码失败
    wrong++;
    return 0;
}

/**
 * @name Joystick_Reflect
 * @brief 摇杆映射
 */
void rmt_Joystick_Reflect(RmtJoystickInfo *js_info)
{
    js_info->left_stick_x = rmt_pow_int16(js_info->left_stick_x, 1.5);
    js_info->left_stick_y = rmt_pow_int16(js_info->left_stick_y, 1.5);
    js_info->right_stick_x = rmt_pow_int16(js_info->right_stick_x, 1.5);
    js_info->right_stick_y = rmt_pow_int16(js_info->right_stick_y, 1.5);
}

void rmt_load_JsBuffer(uint8_t JsSends_datas[],RmtJoystickInfo My_joystick)
{ 
    // LX
    JsSends_datas[0] = My_joystick.left_stick_x >> 8;
    JsSends_datas[1] = My_joystick.left_stick_x & 0xff;
    // LY
    JsSends_datas[2] = My_joystick.left_stick_y >> 8;
    JsSends_datas[3] = My_joystick.left_stick_y & 0xff;
    // RX
    JsSends_datas[4] = My_joystick.right_stick_x >> 8;
    JsSends_datas[5] = My_joystick.right_stick_x & 0xff;
    // RY
    JsSends_datas[6] = My_joystick.right_stick_y >> 8;
    JsSends_datas[7] = My_joystick.right_stick_y & 0xff;
    // Buttons
    // JsSends_datas[8] = (My_joystick.L_But == 0 ? 0x00 : 0xf0) | (My_joystick.R_But == 0 ? 0x00 : 0x0f);
}

int16_t rmt_pow_int16(int16_t a, float pownum)
{
    if (a > 0)
    {
        return (int16_t)pow(a, pownum);
    }
    else if (a < 0)
    {
        return -(int16_t)pow(-a, pownum);
    }
    else
    return 0;
}


void rmt_estimate_js_input()
{
    for (int i = 0; i < 16; i++)
    {
        input_actions[i] = 0;
    }

    // 正面按钮
    for (int i = 0; i < 14; i++)
    {
        if (My_joystick.Keys[i])
        {
            input_actions[i] = 1;
                can_response_flag = 1;//不确该不该置1
        }
    }
    // 拨杆
}