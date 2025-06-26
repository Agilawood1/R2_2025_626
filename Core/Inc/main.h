/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dl_ln.h"
#include "pids.h"
#include "string.h"

#define M3508CLAW_DEBUG

#define USE_PC //串口空闲中断使用上位机的标志位

#define DM_MOTOR_ID_SHOOTER 0x08

#define Dribble_GATE_OUT 0
#define Dribble_GATE_IN 1

#define Shoot_GATE_Fire 0
#define Shoot_GATE_Save 1

/***********    达妙参数    ***************/
#define ShooterDM_Vertical 0.952
#define ShooterDM_Horizontal -0.619

#define MAGAZINE_CLOSE 0
#define MAGAZINE_OPEN 1

extern float ShooterDM_position;    // 达妙电机的转动角度，单位为rad
extern float MagazineDM_position;
extern float ShooterDM_speed;
extern float MagazineDM_speed;

extern float m3508R_max_current;
extern float m3508L_max_current;



void setPWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, float dutyCycle);


extern uint8_t uart_2_rx_Buffer[48];

extern int safe_locks;
extern int safe_guard_timer;

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim5;

/******** 以下变量供 stm32_it 使用 ********/
extern DL_LN_Msg my_recv_msg;
extern float VelocityGain;
extern float RotateVelocityGain;
extern float Vx;
extern float Vy;
extern float Vw;
extern Vec2 STR_MS[4];
extern Vec3 chassis_tf;

extern int uart_recvs_persec;
extern int uart_recvs_persecs[16];
extern uint8_t uart_recvs_persecs_p;


extern int r_3508_speed, l_3508_speed;
extern float test_3508_posi;
extern int dribble_claws_inited, dribble_claw_inorout, dribble_claws_PosiControlable;


extern Pids M3508_PushPull;
extern Pids M3508_Left;
extern Pids M3508_Right;

extern int muzzle_velo_count;
extern float muzzle_velo_seconds;
extern float muzzle_velo;
extern int measurement_done;

extern float VESC_rpm_up;
extern float VESC_rpm_down;

extern uint32_t General_RuntimeCounter;
extern float hori_distance_from_basket;


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define SAFE_GUARD_TIME 200

#define WHEELNUMS 3


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void sendCanTXBuffer(int speed_rpm, float steer_angle_deg, uint8_t Steer_ID);
void sendCanTXBuffer_Confirm(uint8_t Steer_ID);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Shooter_0_Pin GPIO_PIN_7
#define Shooter_0_GPIO_Port GPIOI
#define DribbleConfirm_Pin GPIO_PIN_2
#define DribbleConfirm_GPIO_Port GPIOI
#define DribbleGate_Pin GPIO_PIN_0
#define DribbleGate_GPIO_Port GPIOF
#define ShootGate_Pin GPIO_PIN_1
#define ShootGate_GPIO_Port GPIOF
#define Dribble_Left_ZeroChecker_Pin GPIO_PIN_12
#define Dribble_Left_ZeroChecker_GPIO_Port GPIOH
#define Dribble_Right_ZeroChecker_Pin GPIO_PIN_11
#define Dribble_Right_ZeroChecker_GPIO_Port GPIOH
#define Dribble_Checker_Lower_Pin GPIO_PIN_1
#define Dribble_Checker_Lower_GPIO_Port GPIOA
#define Dribble_Checker_Upper_Pin GPIO_PIN_0
#define Dribble_Checker_Upper_GPIO_Port GPIOA
#define muzzle_velotor_0_Pin GPIO_PIN_2
#define muzzle_velotor_0_GPIO_Port GPIOA
#define muzzle_velotor_0_EXTI_IRQn EXTI2_IRQn
#define muzzle_velotor_1_Pin GPIO_PIN_3
#define muzzle_velotor_1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
