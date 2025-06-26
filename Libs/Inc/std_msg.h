#ifndef STD_MSG_UP70
#define STD_MSG_UP70

// 根据自己的芯片改
#include "stm32f4xx_hal.h"

/**
 * @name 遥控器信息
 * @brief 一个用于传输遥控器状态的类
 * @param l_x, l_y, r_x, r_y: 摇杆数值
 * @param buttons: 按钮被按下，每一个bit都是标志
 */
typedef struct FarCInfo
{
    uint16_t l_x;
    uint16_t l_y;
    uint16_t r_x;
    uint16_t r_y;
    uint16_t buttons;       // 共16个，每个一个bit
}FarCInfo;


/**
 * @name Zigbee网络节点信息
 * @brief 一个包含节点信息的类
 * @param 
 * @param 
 */
typedef struct ZigbeeNodeInfo
{
    char name[8];
}ZigbeeNodeInfo;

/**
 * @name 二维向量
 * @brief 
 * @param x: x分量
 * @param y: y分量
 */
typedef struct Vec2
{
    float x;
    float y;
}Vec2;

/**
 * @name 三维向量
 * @brief 
 * @param x: x分量
 * @param y: y分量
 * @param z: z分量
 */
typedef struct Vec3
{
    float x;
    float y;
    float z;
}Vec3;

typedef struct {
    int id;
    float error;
    float distan;
    float theta;
} SickData;

typedef struct {
    float x;
    float y;
    float fai;
} Vec3_Odom;





#endif