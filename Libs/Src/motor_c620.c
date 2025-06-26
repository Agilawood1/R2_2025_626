#include "motor_c620.h"
#include "bsp_can.h"
#include "logic.h"

int16_t M3508TargetSpd=0;//M3508电机目标速度
int16_t M2006TargetPos=0;//M2006电机目标角度
moto_measure_t M2006; // M2006反馈信息结构体

moto_measure_t M3508s[4]; // M3508反馈信息结构体

uint16_t MyId;

float targ_current_1;
float preheated_time = 0;


/**
 * @name C620 驱动初始化
 */
void motor_c620_init(CAN_HandleTypeDef *hcan)
{
	for (int i = 0; i < 4; i++)
	{
		M3508s[i].my_can = hcan;
		M3508s[i].motor_id = 513 + i;
	}
	
	M3508s[0].motor_pid = pids_create_init(9.2, 0.0, 0.0, 0.001, 4000, 0.3, 0);
	M3508s[1].motor_pid = pids_create_init(9.2, 0.0, 0.0, 0.001, 4000, 0.3, 0);
	M3508s[2].motor_pid = pids_create_init(5.4, 1.0, 0, 0.001, 2000, 0.5, 0);
	M3508s[3].motor_pid = pids_create_init(0, 0, 0, 0.002, 2000, 0.5, 0);
}

/**
 * @name C620 设置RPM
 * @details 只是计算了RPM对应的电流，还得发送
 */
void motor_c620_set_rpm(int motor_rpm, int current_lim, int motor_id)
{
	float targ_current_temp = M3508s[motor_id].motor_pid.calc_output(&M3508s[motor_id].motor_pid, (motor_rpm - M3508s[motor_id].speed_rpm), 10000);
	
	if (targ_current_temp > 10000)
	{
		targ_current_temp = 10000;
	}
	if (targ_current_temp < -10000)
	{
		targ_current_temp = -10000;
	}

	if (targ_current_temp > current_lim)
	{
	targ_current_temp = current_lim;
	}
	if (targ_current_temp < -current_lim)
	{
	targ_current_temp = -current_lim;
	}
	
	// 限制爬坡率
	if (targ_current_temp - M3508s[motor_id].targ_current > 1500)
	{
		M3508s[motor_id].targ_current += 1500;
	}
	else if (targ_current_temp - M3508s[motor_id].targ_current < -1500)
	{
		M3508s[motor_id].targ_current -= 1500;
	}
	else
	{
		M3508s[motor_id].targ_current = targ_current_temp;
	}
}

/// @brief M3508 设置位置的函数
/// @param motor_rpm 
/// @param current_lim 
/// @param motor_id 

/// 等会儿要改
float motor_c620_set_posi(int motor_position, int posi_lim, int speed_lim, int motor_id, Pids* M3508_Pid)
{

	if (motor_position > posi_lim)
	{
		motor_position = posi_lim;
	}
	else if (motor_position < -posi_lim)
	{
		motor_position = -posi_lim;
	}

	float targ_speed = M3508_Pid->calc_output(M3508_Pid, motor_position - M3508s[motor_id].total_angle, 8000) * dribble_gain;

	if (targ_speed > speed_lim)
	{
		targ_speed = speed_lim;
	}
	else if (targ_speed < -speed_lim)
	{
		targ_speed = -speed_lim;
	}
	
	motor_c620_set_rpm(targ_speed, 16000, motor_id);
	
	return targ_speed;
}



void motor_c620_preheat(int motor_id)
{
	if (preheated_time < 0.250)
	{
		motor_c620_set_rpm(200, 1000, motor_id);
	}
	else if (preheated_time > 0.250 && preheated_time < 0.500)
	{
		motor_c620_set_rpm(-200, 1000, motor_id);
	}
	else
	{
		// 预热时间为0.5秒
		M3508s[motor_id].preheated = 1;
	}
	
	preheated_time += M3508s[motor_id].motor_pid.delta_t;
}

/**
 * @description: 获取电机反馈信息
 * @param {moto_measure_t} *ptr电机反馈信息结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
void get_moto_measure(moto_measure_t *ptr, uint8_t *Data)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->real_current = (int16_t)(Data[2] << 8 | Data[3]);
	ptr->speed_rpm = ptr->real_current;
	ptr->given_current = (int16_t)(Data[4] << 8 | Data[5]) / -5;
	ptr->hall = Data[6];
	if (ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt--;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}




/**
 * @description: 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
 * @param {moto_measure_t} *ptr电机结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
void get_moto_offset(moto_measure_t *ptr, uint8_t *Data)
{
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->offset_angle = ptr->angle;
}



/**
 * @description: 获取电机转过的总角度
 * @param {moto_measure_t} *p电机反馈信息结构体指针
 * @return {*}无
 */
void get_total_angle(moto_measure_t *p)
{

	int res1, res2, delta;
	if (p->angle < p->last_angle)
	{											// 可能的情况
		res1 = p->angle + 8192 - p->last_angle; // 正转，delta=+
		res2 = p->angle - p->last_angle;		// 反转	delta=-
	}
	else
	{											// angle > last
		res1 = p->angle - 8192 - p->last_angle; // 反转	delta -
		res2 = p->angle - p->last_angle;		// 正转	delta +
	}
	// 不管正反转，肯定是转的角度小的那个是真的
	if (ABS(res1) < ABS(res2))
		delta = res1;
	else
		delta = res2;
	p->total_angle += delta;
	p->last_angle = p->angle;
}




/**
 * @description: 发送电机电流控制
 * @param {FDCAN_HandleTypeDef} *hfdcan使用的FDCAN
 * @param {uint16_t} m2006
 * @param {uint16_t} m3508
 * @return {*}无
 */
void set_moto_current(CAN_HandleTypeDef *hcan, int16_t motor_0, int16_t motor_1, int16_t motor_2, int16_t motor_3)
{
	uint8_t motor_current_data[8] = {0};
	motor_current_data[0] = motor_0 >> 8;
	motor_current_data[1] = motor_0;
	motor_current_data[2] = motor_1 >> 8;
	motor_current_data[3] = motor_1;

	motor_current_data[4] = motor_2 >> 8;
	motor_current_data[5] = motor_2;
	motor_current_data[6] = motor_3 >> 8;
	motor_current_data[7] = motor_3;

	bspcan_C620_send_msg(hcan, motor_current_data, 8, 0);
}





/**
 * @description: FDCAN接收中断回调函数
 * @param {FDCAN_HandleTypeDef} *hfdcan
 * @param {uint32_t} RxFifo0ITs
 * @return {*}
 */
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
// 	if (hfdcan == &hfdcan1)
// 	{
// 		uint8_t Data[8];
// 		FDCAN_RxHeaderTypeDef Fdcan1RxHeader;
// 		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &Fdcan1RxHeader, Data);
// 		switch (Fdcan1RxHeader.Identifier)
// 		{
// 		case 0X201:
// 			M2006.msg_cnt++ <= 50 ? get_moto_offset(&M2006, Data) : get_moto_measure(&M2006, Data);
// 		break;
// 		case 0X202:
// 			M3508.msg_cnt++ <= 50 ? get_moto_offset(&M3508, Data) : get_moto_measure(&M3508, Data);
// 		break;
// 		}
// 	}
// }
