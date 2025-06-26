#ifndef LOGIC_H
#define LOGIC_H
#include "std_msg.h"

#define STATE_MANAGER_PERIOD_MS 5

/******************     运球    *********************** */
#define Claw_Dribble_Checker_Down   HAL_GPIO_ReadPin(Dribble_Checker_Lower_GPIO_Port, Dribble_Checker_Lower_Pin)
#define Claw_Dribble_Checker_Up     HAL_GPIO_ReadPin(Dribble_Checker_Upper_GPIO_Port, Dribble_Checker_Upper_Pin)

#define M3508CLAW_BLOCK_L !HAL_GPIO_ReadPin(Dribble_Left_ZeroChecker_GPIO_Port, Dribble_Left_ZeroChecker_Pin)
#define M3508CLAW_BLOCK_R !HAL_GPIO_ReadPin(Dribble_Right_ZeroChecker_GPIO_Port, Dribble_Right_ZeroChecker_Pin)

#define M3508CLAW_DribbleConf       !HAL_GPIO_ReadPin(DribbleConfirm_GPIO_Port, DribbleConfirm_Pin)
#define M3508CLAW_DribbleUnconf     HAL_GPIO_ReadPin(DribbleConfirm_GPIO_Port, DribbleConfirm_Pin)

#define L_MECHBIAS 0
#define R_MECHBIAS 0


#define DRIBBLE_END_DELAYTIME_MS 500       // 比赛模式下，运球后多少时间开始装弹
#define MAGELOADING_END_DELAYTIME_MS 500       // 比赛模式下，运球后多少时间开始装弹


/****************       延时器      ********************** */
#define DELAYER(x, y)   if(Logic_delayer_run(delta_ms, x)) {y++;}
#define ROBOWAIT_THENGO(x, y) RobotState = ROBOSTATE_WAIT;RobotPreState = x;Logic_GeneralDelaySet(y)

#define SHOOT_DELAYER_ID 1
#define DRRIBLE_DELAYER_ID 2
#define GUNLOAD_DELAYER_ID 3
#define FIRE_DELAYER_ID 4
#define GENERAL_DELAYER_ID 15



#define MUZZLE_CHECKER_LEN 0.54   // 54cm 即 0.54m




/// @brief 用于解读遥控器行为的枚举变量
typedef enum ACTS_
{
    ACTION_CLAW_OPEN,
    ACTION_CLAW_CLOSE,
    ACTION_CLAWM3508_UP,
    ACTION_CLAWM3508_DOWN,
    ACTION_DRIBBLE,
    ACTION_SHOOT,
    ACTION_PULL = 7,
    ACTION_PUSH = 9,
}ACTS_;

/// @brief 夹爪的状态
typedef enum DRB_
{
    DRIBBLE_CLAW_IN,
    DRIBBLE_CLAW_OUT,
}DRB_;

/// @brief 机器人的状态：作用请见Readme
typedef enum RBS_
{
    ROBOSTATE_INITING,           // 正在初始化
    ROBOSTATE_IDLE,             // 空闲状态
    ROBOSTATE_DRIBBLING,           // 正在运球
    ROBOSTATE_MAGELOADING,          // 正在装弹

    /*****  三种行动模式   ********/
    ROBOSTATE_MANUAL_MOVING,        // 由遥控器控制
    ROBOSTATE_DIRECT_MOVING,        // 自动直线路径
    ROBOSTATE_AUTOMATIC_MOVING,     // 全自动行动

    ROBOSTATE_SHOOTING,     // 正在发射
    ROBOSTATE_WAIT,
}ROBOST;

/// @brief 机器人的模式：作用请见Readme
typedef enum RBM_
{
    ROBOMODE_MATCH,             // 比赛模式
	  ROBOMODE_MATCH_AUTO,             // 自动比赛模式
    ROBOMODE_MANUAL,            // 手动测试模式
    ROBOMODE_CANNONTEST,        // 大炮测试模式
}ROBOMD;

/**************         全局状态管理器        ********************/
void Logic_StateRunner();
void Logic_StateTransmitor();

/**************         动作序列封装        ********************/

void Logic_InputResp();
int Logic_GunLoad(int delta_ms);
int Logic_Dribble(int delta_ms);
int Logic_TestFire(int delta_ms);
int Logic_DircMoving(Vec3 targ_tf, Vec3 real_tf);


/**************     灯带标志位      ****************************/
extern int carinit_flag, carwait_flag;               // 初始化标志
extern int idle_flag, drbl_flag, gunload_flag, mving_flag;                  // 空闲标志


/**************         各种状态标志位        ********************/


/**************         各种变量        ********************/
extern uint8_t can_response_flag;
extern uint8_t input_actions[16];         // 不同的输入响应
extern int left_3508_posi;
extern int right_3508_posi;

extern Vec3 shoottarg_tf;

// extern float dribble_gain;
extern int input_cooldown_counting;

extern float init_progress;     // 初始化进度

/*****************      stm32_it中，DEBUG用的FLAGS      ***************************/
extern int YUNQIU_flag, ZHUANGDAN_flag, FaShe_flag, RUNNING_flag;
extern ROBOST RobotState;
extern ROBOMD RobotMode;     //  机器人所处状态，模式


#endif

