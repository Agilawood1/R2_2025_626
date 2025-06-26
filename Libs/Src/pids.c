// filePath: pids.c

#include "pids.h"

float limit_ab(float a, float limit)
{
    if (a > limit)
    {
        return limit;
    }
    else if (a < -limit)
    {
        return -limit;
    }
    else
    {
        return a;
    }
}

/// @brief 初始化，并返回一个PIDS结构体
/// @param Kp 
/// @param Ki 
/// @param Kd 
/// @param delta_t 
/// @param inte_limit 
/// @param delta_filter_rate 
/// @param reverse  是否反转输出，有些pid需要反转输出
/// @return 一个PIDS结构体
Pids pids_create_init(float Kp, float Ki, float Kd, float delta_t, float inte_limit, float delta_filter_rate, int reverse)
{
    Pids new_pids = {
        .Kp = Kp, .Ki = Ki, .Kd = Kd, .integral_limit = inte_limit, .delta_filter_rate = delta_filter_rate, 
        .delta_t = delta_t, .reverse = reverse, .output_limit = 0.0f, .control_value = 0.0f,
        .last_error = 0, .prev_error = 0, .integral_errors = 0, .kd_error = 0
    };
    new_pids.calc_output = calc_output;
    new_pids.calc_increment = calc_increment;
    new_pids.calc_output_incremental = calc_output_incremental;

    return new_pids;
}

void pids_change_param(Pids *targ_pid, float Kp, float Ki, float Kd)
{
    targ_pid->Kp = Kp;
    targ_pid->Ki = Ki;
    targ_pid->Kd = Kd;
}

/**
 * @brief 重置PID控制器的状态
 * @param targ_pid PID结构体指针
 */
void pids_reset(Pids *targ_pid)
{
    targ_pid->integral_errors = 0;
    targ_pid->last_error = 0;
    targ_pid->prev_error = 0;
    targ_pid->kd_error = 0;
    targ_pid->control_value = 0;
}

float calc_output(struct pids* targ_pid, float error, float output_lim)
{
    float pid_output;

    // 为Kd计算经过一阶低通滤波的误差
    targ_pid->kd_error = targ_pid->kd_error + targ_pid->delta_filter_rate * (error - targ_pid->kd_error);
    
    // 累加积分项
    targ_pid->integral_errors += targ_pid->Ki * error * targ_pid->delta_t;

    // 如果用户配置了积分限幅，作限幅
    if (targ_pid->integral_limit > 0)
    {
        targ_pid->integral_errors = limit_ab(targ_pid->integral_errors, targ_pid->integral_limit);
    }

    // 计算普通PID
    pid_output = targ_pid->Kp * error + targ_pid->integral_errors + targ_pid->Kd * ((targ_pid->kd_error - targ_pid->last_error) / targ_pid->delta_t);

    // 记录误差项，用于下次计算
    targ_pid->prev_error = targ_pid->last_error;
    targ_pid->last_error = error;

    
    // 如果用户配置了反向，作反向输出
    if (targ_pid->reverse)
    {
        pid_output = -pid_output;
    }
    
    // 应用输出限幅（如果配置了限幅）
    if (output_lim > 0)
    {
        pid_output = limit_ab(pid_output, output_lim);
    }

    return pid_output;
}

/**
 * @brief 计算增量式PID的输出增量
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @return 控制量的增量
 */
float calc_increment(struct pids* targ_pid, float error)
{
    float increment;
    
    // 计算增量式PID
    increment = targ_pid->Kp * (error - targ_pid->last_error) +
                targ_pid->Ki * error * targ_pid->delta_t +
                targ_pid->Kd * (error - 2 * targ_pid->last_error + targ_pid->prev_error) / targ_pid->delta_t;
    
    // 记录误差项，用于下次计算
    targ_pid->prev_error = targ_pid->last_error;
    targ_pid->last_error = error;
    
    // 如果用户配置了反向，作反向输出
    if (targ_pid->reverse)
    {
        increment = -increment;
    }
    
    return increment;
}

/**
 * @brief 计算增量式PID的输出（内部维护累加值和限幅），带微分滤波器
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @return 经过累加和限幅后的控制量
 */
float calc_output_incremental(struct pids* targ_pid, float error, float output_lim)
{
    // 为增量式PID的微分项添加低通滤波
    targ_pid->kd_error = targ_pid->kd_error + targ_pid->delta_filter_rate * (error - targ_pid->kd_error);
    
    // 计算控制量增量，使用滤波后的误差进行微分项计算
    float increment = targ_pid->Kp * (targ_pid->kd_error - targ_pid->last_error) +
                      targ_pid->Ki * error * targ_pid->delta_t +
                      targ_pid->Kd * (targ_pid->kd_error - 2 * targ_pid->last_error + targ_pid->prev_error) / targ_pid->delta_t;
    
    // 记录误差项，用于下次计算
    targ_pid->prev_error = targ_pid->last_error;
    targ_pid->last_error = targ_pid->kd_error;  // 保存滤波后的误差，而非原始误差
    
    // 更新控制量
    targ_pid->control_value += increment;
    
    // 应用输出限幅（如果配置了限幅）
    if (output_lim > 0)
    {
        targ_pid->control_value = limit_ab(targ_pid->control_value, output_lim);
    }
    
    // 如果用户配置了反向，作反向输出
    if (targ_pid->reverse)
    {
        targ_pid->control_value = -targ_pid->control_value;
    }
    
    return targ_pid->control_value;
}