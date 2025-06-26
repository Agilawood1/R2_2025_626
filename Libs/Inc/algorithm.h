#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "rmt_stick.h"
#include "std_msg.h"

#define STEER_WHEEL_RADIO 0.0871
#define STEER_WHEEL_C 0.54636
#define STEER_WHEEL_60RPM_ERPM 1520
#define STEER_WHEEL_POLE_PAIRS 21
#define MATH_PI 3.1415926

#define STEER_LinkConfirm 10010
#define STEER_Control 10000

/***********    投篮参数    *****************/
#define DEG_2_RAD 0.017453292           //  (角度 转 弧度) 的系数
#define RAD_2_DEG 57.29578049           //  (角度 转 弧度) 的系数

#define SQRT_3 1.7320508                //  根号三
#define BASKET_HEIGHT 2.63          //  篮筐高度：2.63米
#define GRAVITY_G 9.7949



#define MeterPerSec_2_ERPM_ (1 / STEER_WHEEL_C) * 60 * STEER_WHEEL_POLE_PAIRS
#define MeterPerSec_2_ERPM 2306.1718

#define DM_CLAW_4_5_OFFSET 0.04            // 越大越往上
#define DM_CLAW_6_7_OFFSET 0.15            // 越大越往上
#define DM_CLAW_GENERAL_OFFSET 0.48      // 越大越往上

#define M3508_CLAW_HIGHEST -310000      // 注意，越负越往上
#define M3508_CLAW_LOWEST -1000          // 不为0，防止过热

void algo_get_steerBetter_vec(int velo_now, float angleDeg_now, int* velo_targ, float* angleDeg_targ);
void algo_calc_steer_vecs_4(float V_x, float V_y, float V_w, Vec2 Str_Ms[]);
void algo_calc_steer_vecs_3(float V_x, float V_y, float V_w, Vec2 Str_Ms[]);
void algo_calc_steer_vecs_3_test_forward(Vec2 Str_Ms[]);

void algo_vec2_add(Vec2* vec_1, Vec2* vec_2);
void algo_vec2_add_xy(Vec2* vec_1, float x, float y);
Vec2 algo_vec2_add_tonew(Vec2 vec_1, Vec2 vec_2);
Vec3 algo_vec3_add_tonew(Vec3 vec_1, Vec3 vec_2);
Vec3 algo_norm(Vec3 v);
float algo_vec3_distance(Vec3 vec1, Vec3 vec2);

void algo_vec2_multiply(Vec2* vec, float k);
void algo_vec3_multiply(Vec3 *vec, float k);

void algo_js_to_chassis_vec(Vec3 *chassis_vec, RmtJoystickInfo js_info);
void algo_polesys_to_vec2(Vec2* vec);
Vec3 algo_OdomVec_to_LocalVec(Vec3 Odom_Vec);
void algo_chassis_vec_constrain(Vec3 *chassis_vec, float limit);

Vec2 algo_shooter_calc_fireparam(float hori_dis_from_basket);


extern Vec2 unit_x_vec2;
extern Vec2 unit_y_vec2;
extern Vec2 unit_45_vec2;
extern Vec2 unit_135_vec2;
extern Vec2 unit_225_vec2;
extern Vec2 unit_315_vec2;
extern float algo_steerangles_preserve[4];

#endif