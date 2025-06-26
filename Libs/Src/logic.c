#include "logic.h"
#include "motor_dm.h"
#include "main.h"
#include "motor_VESC.h"
#include "motor_c620.h"
#include "algorithm.h"


int left_3508_posi = 0;
int right_3508_posi = 0;
Vec3 shoottarg_tf;      // 目标发射位置


int drbl_delay_ms = 150;
ROBOST RobotState = ROBOSTATE_INITING, RobotPreState = ROBOSTATE_IDLE;     //  机器人所处状态
ROBOMD RobotMode = ROBOMODE_MANUAL;         //  机器人所处模式

/**************     灯带标志位      ********************/
int carinit_flag = 0, carwait_flag = 0;               // 初始化标志
int idle_flag = 0;                  // 空闲标志
int drbl_flag = 0, gunload_flag = 0;                  // 运球标志位
int mving_flag = 0;


/************           用于状态转移的标志位        *****************/
int dribble_ok_flag = 0, gunload_ok_flag = 0, dir_moving_ok_flag = 0;
int wait_ok_flag = 0;




uint8_t input_actions[16]; // 不同的输入响应
int input_cooldown_counting = 0;
uint8_t can_response_flag = 1;

// 延时函数
void Logic_delayer_set(int delay_ms, uint8_t delayer_id);
int Logic_delayer_run(int delta_ms, uint8_t delayer_id);

/**
 * @brief 动作序列
 * @name 装弹
 * @param delta_ms：每次调用本函数的间隔
 * @warning 调用本函数的时候，要求篮球必须在运球机构的上端放着
 * @details 一系列动作
 */

int Logic_GunLoad(int delta_ms)
{
    return 0;
}




/**
 * @brief 动作序列
 * @name 运球
 * @param delta_ms：每次调用本函数的间隔
 * @param dribble_checker：红外是否检测到篮球在面前
 * @details 一系列动作
 */
float dribble_gain = 0.5f;
int Logic_Dribble(int delta_ms)
{
   
    return 0;
}


/**
 * @brief 动作序列
 * @name 测试性发射
 * @param delta_ms：每次调用本函数的间隔
 * @details 如果机器人的状态不是“TestFire”，本操作是无效的
 */
int Logic_TestFire(int delta_ms)
{
    static int testfire_state = 0;
    switch (testfire_state)
    {
        case 0:
        {
            
            // 设置延时器
            Logic_delayer_set(400, TESTFIRE_DELAYER_ID);
            testfire_state++;
            break;
        }
        case 1:
        {
            // 运行延时器，直到进入下一个状态
            DELAYER(TESTFIRE_DELAYER_ID, testfire_state)
            break;
        }
    }
}


/**
 * @brief 通用状态机逻辑延时器
 * @name 逻辑延时器
 * @param delta_ms：每次调用本函数的间隔
 * @details 使用之前使用Logic_GeneralDelaySet设置延迟的毫秒
 */
int Logic_Wait(int delta_ms)
{
    // 运行延时器
    if(Logic_delayer_run(5, GENERAL_DELAYER_ID))
        return 1;
    else
        return 0;
}
    

/*******************************************************************************************/

/// @brief 输入响应
/**
 * @details 根据不同的情况，做出不同的 单次 响应
 * @note 有冷却时间
 */
void Logic_InputResp()
{
    // 关闭夹爪 的动作
    if (input_actions[ACTION_CLAW_CLOSE])
    {
        input_actions[ACTION_CLAW_CLOSE] = 0;
    }

    // 开启夹爪 的动作
    if (input_actions[ACTION_CLAW_OPEN])
    {
        input_actions[ACTION_CLAW_OPEN] = 0;
    }

    // 提高爪子 的动作
    if (input_actions[ACTION_CLAWM3508_UP])
    {
        input_actions[ACTION_CLAWM3508_UP] = 0;
    }

    // 降低爪子 的动作
    if (input_actions[ACTION_CLAWM3508_DOWN])
    {
        input_actions[ACTION_CLAWM3508_DOWN] = 0;
    }

    // 开始运球 的动作
    if (input_actions[ACTION_DRIBBLE])
    {
        // 在启动 “系列动作” 之后，响应会被关闭，避免影响动作序列，产生错误
        YUNQIU_flag = 1;
        can_response_flag = 0;

        input_actions[ACTION_DRIBBLE] = 0;
    }

    // 开始射击 的动作
    if (input_actions[ACTION_SHOOT])
    {
        // 在启动 “系列动作” 之后，响应会被关闭，避免影响动作序列，产生错误
        FaShe_flag = 1;
        can_response_flag = 0;

        input_actions[ACTION_SHOOT] = 0;
    }

    // 打开弹舱 的动作
    if (input_actions[ACTION_PULL])
    {
        input_actions[ACTION_PULL] = 0;
    }

    // 拉取球 的动作
    if (input_actions[ACTION_PUSH])
    {
        input_actions[ACTION_PUSH] = 0;
    }
}


/// @brief 直接前往
/**
 * @details 
 * @note 
 */
int Logic_DircMoving(Vec3 targ_tf, Vec3 real_tf)
{
    Vec3 nega_real_tf;
    nega_real_tf.x = -real_tf.x;
    nega_real_tf.y = -real_tf.y;
    nega_real_tf.z = -real_tf.z;

    Vec3 speed_vec = algo_vec3_add_tonew(targ_tf, nega_real_tf);
    algo_vec3_multiply(&speed_vec, 0.25);

    speed_vec.x = limit_abs(speed_vec.x, 0.35);
    speed_vec.y = limit_abs(speed_vec.y, 0.35);
    speed_vec.z = limit_abs(speed_vec.z, 0.25);

    Vx = speed_vec.x;
    Vy = speed_vec.y;
    Vw = speed_vec.z;

    real_tf.z = 0;
    targ_tf.z = 0;
    if (algo_vec3_distance(targ_tf, real_tf) < 0.05)
    {
        return 1;
    }
    return 0;
}


/************************       延时函数        *******************************************/
int delayer_counter[16] = {0};
/**
 * @name 延时函数设置
 * @param
 * @details
 */
void Logic_delayer_set(int delay_ms, uint8_t delayer_id)
{
    delayer_counter[delayer_id] = delay_ms;
}


/**
 * @name 延时函数运行
 * @param
 * @details
 */
int Logic_delayer_run(int delta_ms, uint8_t delayer_id)
{
    delayer_counter[delayer_id] -= delta_ms;
    if (delayer_counter[delayer_id] <= 0)
    {
        delayer_counter[delayer_id] = 0;
        return 1;
    }
    else
        return 0;
}

/**
 * @name 通用延时函数设置
 * @param 
 * @details
 */
void Logic_GeneralDelaySet(int delay_ms)
{
    delayer_counter[GENERAL_DELAYER_ID] = delay_ms;
}

void Logic_Clear_WSflag()
{
    carinit_flag = 0;
    carwait_flag = 0;               // 初始化标志
    idle_flag = 0;                  // 空闲标志
    drbl_flag = 0;                  // 运球标志位
    mving_flag = 0;
    gunload_flag = 0;
}


/**********************     有限状态机：需要用户自定义    *************************/
/**
 * @name 状态运行机
 */
void Logic_StateRunner()
{
    Logic_Clear_WSflag();
    switch (RobotState)
    {
        /************************   状态：等待    *******************************/
        case ROBOSTATE_WAIT:
        {
            carwait_flag = 1;
            wait_ok_flag = Logic_Wait(STATE_MANAGER_PERIOD_MS);
            break;
        }
        /************************   状态：初始化    *******************************/
        case ROBOSTATE_INITING:
        {
            carinit_flag = 1;
            break;
        }
        /************************   状态：空闲    *******************************/
        case ROBOSTATE_IDLE:
        {
            idle_flag = 1;
            break;
        }
        /************************   状态：运球    *******************************/
        case ROBOSTATE_DRIBBLING:
        {
            drbl_flag = 1;
            dribble_ok_flag = Logic_Dribble(STATE_MANAGER_PERIOD_MS);
            break;
        }
        /************************   状态：装弹    *******************************/
        case ROBOSTATE_MAGELOADING:
        {
            gunload_flag = 1;
            gunload_ok_flag = Logic_GunLoad(STATE_MANAGER_PERIOD_MS);
            break;
        }
        /************************   状态：直接行走    *******************************/
        case ROBOSTATE_DIRECT_MOVING:
        {
            mving_flag = 1;
            dir_moving_ok_flag = Logic_DircMoving(shoottarg_tf, chassis_tf);
            break;
        }
    }
}


/**
 * @name 状态转移机
 */
void Logic_StateTransmitor()
{
    switch (RobotState)
    {
        /************************   状态：初始化    *******************************/
        case ROBOSTATE_INITING:
        {
            if (1)      // 如果夹爪初始化成功了，进入下一个状态
            {
                RobotState = ROBOSTATE_IDLE;
            }
            break;
        }
        /************************   状态：空闲    *******************************/
        case ROBOSTATE_IDLE:
        {
            switch (RobotMode)
            {
                /****   模式：比赛    ****/
                case ROBOMODE_MATCH:
                {
                    if (RUNNING_flag)
                    {
                        RobotState = ROBOSTATE_DRIBBLING;
                        RUNNING_flag = 0;
                    }
                    
                    break;
                }
                /****   模式：手控调试    ****/
                case ROBOMODE_MANUAL:
                {
                    if (YUNQIU_flag)
                    {
                        RobotState = ROBOSTATE_DRIBBLING;
                        YUNQIU_flag = 0;
                    }
                    break;
                }
                /****   模式：大炮标定    ****/
                case ROBOMODE_CANNONTEST:
                {
                    
                    break;
                }
            }
            break;
        }
        /************************   状态：运球    *******************************/
        case ROBOSTATE_DRIBBLING:
        {
            switch (RobotMode)
            {
                /****   模式：比赛    ****/
                case ROBOMODE_MATCH:
                {
                    if (dribble_ok_flag)
                    {
                        ROBOWAIT_THENGO(ROBOSTATE_MAGELOADING, DRIBBLE_END_DELAYTIME_MS);        // 等待一下再进入下一个状态
                        dribble_ok_flag = 0;
                    }
                    
                    break;
                }
                /****   模式：手控调试    ****/
                case ROBOMODE_MANUAL:
                {
                    if (dribble_ok_flag)
                    {
                        RobotState = ROBOSTATE_IDLE;        // 返回空闲状态
                        dribble_ok_flag = 0;
                    }
                    break;
                }
                /****   模式：大炮标定    ****/
                case ROBOMODE_CANNONTEST:
                {
                    
                    break;
                }
            }
            break;
        }
        /************************   状态：装弹    *******************************/
        case ROBOSTATE_MAGELOADING:
        {
            switch (RobotMode)
            {
                /****   模式：比赛    ****/
                case ROBOMODE_MATCH:
                {
                    if (gunload_ok_flag)
                    {
                        ROBOWAIT_THENGO(ROBOSTATE_DIRECT_MOVING, MAGELOADING_END_DELAYTIME_MS);        // 等待一下再进入下一个状态
                        gunload_ok_flag = 0;
                    }
                    
                    break;
                }
                /****   模式：手控调试    ****/
                case ROBOMODE_MANUAL:
                {
                    if (gunload_ok_flag)
                    {
                        RobotState = ROBOSTATE_IDLE;        // 返回空闲状态
                        gunload_ok_flag = 0;
                    }
                    break;
                }
                /****   模式：大炮标定    ****/
                case ROBOMODE_CANNONTEST:
                {
                    
                    break;
                }
            }
            break;
        }
        /************************   状态：直接行走    *******************************/
        case ROBOSTATE_DIRECT_MOVING:
        {
            switch (RobotMode)
            {
                /****   模式：比赛    ****/
                case ROBOMODE_MATCH:
                {
                    if (dir_moving_ok_flag)
                    {
                        ROBOWAIT_THENGO(ROBOSTATE_IDLE, MAGELOADING_END_DELAYTIME_MS);        // 等待一下再进入下一个状态
                        dir_moving_ok_flag = 0;
                    }
                    
                    break;
                }
                /****   模式：手控调试    ****/
                case ROBOMODE_MANUAL:
                {
                    if (dir_moving_ok_flag)
                    {
                        RobotState = ROBOSTATE_IDLE;        // 返回空闲状态
                        dir_moving_ok_flag = 0;
                    }
                    break;
                }
                /****   模式：大炮标定    ****/
                case ROBOMODE_CANNONTEST:
                {
                    
                    break;
                }
            }
            break;
        }
        /************************   状态：等待    *******************************/
        case ROBOSTATE_WAIT:
        {
            if (wait_ok_flag)
            {
                RobotState = RobotPreState;        // 前往目标状态
                wait_ok_flag = 0;
            }
            break;
        }
    }
}



// 将4个字节转换为float类型
float bytesToFloat(uint8_t* bytes) {
    float result;
    memcpy(&result, bytes, sizeof(float));
    return result;
}
// 计算帧头校验
uint8_t calculateFrameHead(uint8_t* data, int length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}
// 计算帧尾校验
uint8_t calculateFrameTail(uint8_t* data, int length) {
    uint8_t xor_result = 0;
    for (int i = 0; i < length; i++) {
        xor_result ^= data[i];
    }
    return xor_result;
}
/// @brief 工控机串口数据解码
/// @param vec 
/// @param buf 
/// @return 
// 解包控制类数据的函数
int Logic_DecodeHostMsg(uint8_t* buf) {
    // 假设数据包长度足够，这里不做详细的长度检查
    uint8_t frame_head = buf[0];
    uint8_t frame_type = buf[1];

    // 检查帧头校验
    uint8_t calculated_head = calculateFrameHead(&buf[1], 14);
    if (frame_head != calculated_head) {
        return 0; // 帧头校验失败
    }

    // 检查帧尾校验
    uint8_t frame_tail = buf[15];
    uint8_t calculated_tail = calculateFrameTail(&buf[1], 14);
    if (frame_tail != calculated_tail) {
        return 0; // 帧尾校验失败
    }
    

    switch (frame_type) {
        case 0x02:  // 工控机请求控制权
        {
            // 数据位必须为0xf0
            if (buf[2] != 0xf0) {
                return 0;
            }
            // 工控机请求控制权的处理逻辑
            RobotMode = ROBOMODE_MATCH_AUTO;
            break;
        }
            
        case 0x03:  // 工控机送回控制权
        {
            // 数据位必须为0xf0
            if (buf[2] != 0xf0) {
                return 0;
            }
            // 工控机送回控制权的处理逻辑
            RobotMode = ROBOMODE_MATCH;
            break;
        }
            
        case 0x10: // 底盘移动
        {
            if (RobotMode == ROBOMODE_MATCH_AUTO) {
            Vx = bytesToFloat(&buf[2]);
            Vy = bytesToFloat(&buf[6]);
            Vw = bytesToFloat(&buf[10]);
            break;}
        
        }
            
        case 0x11: // 发射俯仰角控制
        {
            if (RobotMode == ROBOMODE_MATCH_AUTO) {
            ShooterDM_position = bytesToFloat(&buf[2]);
            break;}
        }
            
        case 0x12: // 摩擦轮速度控制
        {
            if (RobotMode == ROBOMODE_MATCH_AUTO) {
            MagazineDM_position = bytesToFloat(&buf[2]);
            break;}
        }
            
        case 0x13:  // 发射
        {
            if (RobotMode == ROBOMODE_MATCH_AUTO) {
            // 数据位必须为0xf1
            if (buf[2] != 0xf1) {
                return 0;
            }
            // 发射操作
            FaShe_flag = 1;
            can_response_flag = 0;
            break;}
        }
            
        default:
            return 0; // 未知的帧类型
    }

    return 1; // 解包成功
}


