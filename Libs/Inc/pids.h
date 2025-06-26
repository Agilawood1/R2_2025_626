// filePath: pids.h

#ifndef PIDS_H
#define PIDS_H

// 必须初始化的参数：Kp, Ki, Kd, delta_t, inte_limit, delta_filter_rate
/**
 * @name Pids
 * @brief 普通的PID结构体
 * @note 初始化之后，使用calc函数计算输出
 * @param delta_filter_rate：Kd低通滤波系数
 * @param integral_limit：积分限幅
 * @param output_limit：输出限幅
 */
typedef struct pids
{
    float Kp;
    float Ki;
    float Kd;
    float delta_t;
    int reverse;

    float integral_errors;
    float last_error;
    float prev_error; // 新增：保存上上次的误差，用于增量式PID计算

    float integral_limit;
    float output_limit; // 新增：输出限幅
    float kd_error;
    float delta_filter_rate;

    float control_value; // 新增：控制量累加值
    
    float (*calc_output)(struct pids* targ_pid, float error, float output_lim);
    float (*calc_increment)(struct pids* targ_pid, float error); // 增量式PID计算函数
    float (*calc_output_incremental)(struct pids* targ_pid, float error, float output_lim); // 新增：带内部累加的增量式PID计算函数
}Pids;

Pids pids_create_init(float Kp, float Ki, float Kd, float delta_t, float inte_limit, float delta_filter_rate, int reverse);
float calc_output(struct pids* targ_pid, float error, float output_lim);
float calc_increment(struct pids* targ_pid, float error);
float calc_output_incremental(struct pids* targ_pid, float error, float output_lim); // 新增：带内部累加的增量式PID计算函数声明
void pids_change_param(Pids *targ_pid, float Kp, float Ki, float Kd);
void pids_reset(Pids *targ_pid); // 新增：重置PID状态函数声明

#endif