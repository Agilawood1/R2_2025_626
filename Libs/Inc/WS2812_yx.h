#ifndef WS2812_YX
#define WS2812_YX
// #include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define USE_SPI



#ifdef USE_SPI
#define HIGH_WS2812 248
#define LOW_WS2812 192
#elif USE_PWM
#define HIGH_WS2812 15
#define LOW_WS2812 6
#endif


#define LED_Nums 58

typedef enum WS2812_State
{
    WS2812_fill,
    WS2812_expand,
    WS2812_flow,
    WS2812_clear,
    WS2812_wonder,
    WS2812_2c_gradflow,
}WS2812_State;

/**
 * @name WS2812灯带 状态描述结构体
 * @param priority：越高，该状态优先级越高
 * @param RGB：颜色
 * @param counter_period: 速度，默认为 1，代表减速多少倍（比如设置为2，那么每两次调用才会生效）
 * @param counter：内部变量
 */
typedef struct WS2812_StateDescription
{
    WS2812_State state;
    uint8_t priority;
    uint8_t R;
    uint8_t G;
    uint8_t B;

    uint8_t R2;
    uint8_t G2;
    uint8_t B2;

    uint8_t R3;
    uint8_t G3;
    uint8_t B3;

    uint8_t band_wide;
    float Delit_rate;
    uint16_t counter_period;
    
    int counter;

}WS2812_StateDescription;

typedef struct WS2812_StateLink
{
    int* link_flag;
    WS2812_StateDescription* targ_state;

}WS2812_StateLink;

void RGB_SetOne_Color(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B);
void WS2812_InitBuffer();
void WS2812_Flows(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate);
void WS2812_Expand_From_Center(uint8_t R, uint8_t G, uint8_t B);
void WS2812_Fill(uint8_t R, uint8_t G, uint8_t B);
void WS2812_TwoColorGradientFlow(uint8_t R1, uint8_t G1, uint8_t B1, uint8_t R2, uint8_t G2, uint8_t B2);

/***********            专供状态变量的函数            ******************/
void WS2812_Flows_St(WS2812_StateDescription *state);
void WS2812_Expand_From_Center_St(WS2812_StateDescription *state);
void WS2812_Fill_St(WS2812_StateDescription *state);
void WS2812_TwoColorGradientFlow_St(WS2812_StateDescription *state);



void WS2812_Angles(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate,float angle, int reverse, int bias);       //  0-360度映射灯珠0-N
void WS2812_Angles_Add(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate,float angle, int reverse, int bias);

void WS2812_Wonder(float intensity);
__weak void WS2812_Refresh();

/*          状态监测器          */
void WS2812_AddStateLink(int* link_flag, WS2812_StateDescription *targ_state);
void WS2812_Set_State(WS2812_StateDescription *targ_state);
void WS2812_State_Handler();        // 放在持续更新处

extern WS2812_StateLink WS2812_my_links[64];
extern uint8_t WS2812buf2send[LED_Nums][24];
// extern int8_t Led_Center_id;

#ifdef USE_WS2812_PREDEFINES
extern WS2812_StateDescription Yellow_Flow;
extern WS2812_StateDescription LightPurple_Expand;
extern WS2812_StateDescription Yellow_Fill;
extern WS2812_StateDescription LightBlue_Flow;
extern WS2812_StateDescription Wonderful;
extern WS2812_StateDescription Pure_Blue_HighPriority;
extern WS2812_StateDescription Pure_Red_HighPriority;
extern WS2812_StateDescription Pure_Green_HighPriority;
extern WS2812_StateDescription Pure_Black_HighPriority;
extern WS2812_StateDescription Pure_White_HighPriority;
#endif
// extern WS2812_StateDescription Yellow_Flow;
// extern WS2812_StateDescription Yellow_Flow;
// extern WS2812_StateDescription Yellow_Flow;
// extern WS2812_StateDescription Yellow_Flow;


//extern int targ_link_id;
//int targ_priority;
// extern int test_int;
/**********************     以下是专供本工程的状态      ***************************/
extern WS2812_StateDescription NJUST_Purple_Expand;
extern WS2812_StateDescription NJUST_Purple_Flow;
extern WS2812_StateDescription UP70_YELLOW_Fill;
extern WS2812_StateDescription Wonderful;
extern WS2812_StateDescription Test_2c_gradflow;
#endif