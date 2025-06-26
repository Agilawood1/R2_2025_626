/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rmt_stick.h"
#include "algorithm.h"
#include "AD7606.h"
#include "delay.h"
#include "string.h"
#include "stm32f4xx_hal_dma.h"
#include "action_OPS.h"
#include "silocator.h"
#include "motor_dm.h"
#include "bsp_can.h"
#include "flash_stm.h"
#include "motor_VESC.h"
#include "WS2812_yx.h"
#include "logic.h"
//dzx添加的库
#include "dzx_lib.h"
//-------------------
#define USE_SICKDATA

// 需要手动控制炮台时启用，调达妙时注释与否都可以调
// #define MANULLY_DEBUG




void CAN_Interrupt_Enable(CAN_HandleTypeDef *can_n);
void sendCanTXBuffer(int speed_rpm, float steer_angle_deg, uint8_t Steer_ID);
void sendCanTXBuffer_Confirm(uint8_t Steer_ID);
void sendMsgToChassis();
// 编码函数：将Vec3数据编码为字节数组
void encodeVec3(const Vec3* vec, uint8_t* buffer);
// 解码函数：从字节数组解码为Vec3数据
void decodeVec3(const uint8_t* buffer, Vec3* vec);

void WS2812_Refresh()
{
  HAL_SPI_Transmit_DMA(&hspi4, (uint8_t*)WS2812buf2send, 24 * (LED_Nums + 1));
}


int can_recvs = 0;
int send_speed_rpm = 1000;
float send_angle_deg = 30;
char print_String[48];
uint8_t vofa_data[24];

// 全局变量记录 上次触发 出膛速度测试 的时间
volatile uint32_t last_pa2_trigger_time = 0;
volatile uint32_t last_pd12_trigger_time = 0;
#define COOLDOWN_TIME_MS 1000 // 冷却时间 1 秒



// 总计时器
uint32_t General_RuntimeCounter = 0;

// 车体姿态、里程计姿态、激光修正姿态
Vec3 chassis_tf;
Vec3 odom_tf;
Vec3 silocator_tf = {0, 0, 0};

// 车体里程计
Odometer chassis_odom;

// 接收到的 Zigbee 数据包
DL_LN_Msg my_recv_msg;

uint8_t sendbuf[8] = {0};
float acc[8];

// 安全锁
int safe_locks = 0;
// 安全员倒计时
int safe_guard_timer = 0;

// // 由遥控器接收到的底盘动作
int check_can_id = 0;

// 速度增益
float VelocityGain = 1.0;
float RotateVelocityGain = 1.0;

// CAN总线接收次数
int can_1_recvtimes = 0;
int can_2_recvtimes = 0;
int can_recvtimes = 0;


// 运球3508最大电流
float m3508R_max_current = 0;
float m3508L_max_current = 0;



/***************    状态变量    *********************/
int dribble_claw_inorout = DRIBBLE_CLAW_IN;     // 开始爪子在里边
int magazine_OpenorClose = MAGAZINE_CLOSE;      // 弹匣默认关闭


/*******************    所有的初始化状态变量    *********************/    // 如果没有初始化就做动作，会产生可怕的后果！





/****************     摩擦轮转速设置    ***********************/
float VESC_rpm_up = 0;
float VESC_rpm_down = 0;
int read_VESC_id = 0;


/***********************    Debug用的标志    ***********************/
int Claw_Close_Flag = 0;
int Claw_Open_Flag = 0;

int Claw_Up_Flag = 0;
int Claw_Down_Flag = 0;
int Claw_Middle_Flag = 0;

int up_debug_flag = 0;
int down_debug_flag = 0;


float hori_distance_from_basket = 0;

int temp_1, temp_2;
float temp_float;

/***********************    达妙用的数据    ***********************/

float ShooterDM_position = 0.0f;
float MagazineDM_position = 0.015f;
float ShooterDM_speed = DM_ROUND_PERSEC / 2;
float MagazineDM_speed = DM_TWOROUND_PERSEC;

float DM_Left_position = 0;
float DM_Left_speed = 1.57;
float DM_Right_position = 0;
float DM_Right_speed = 1.57;

// 二、四、六号电机
uint8_t Two_buf[8] = {0};
uint8_t Four_buf[8] = {0};
uint8_t Six_buf[8] = {0};

// 炮筒、弹匣达妙电机
uint8_t ShooterDM_buf[8] = {0};
uint8_t MagazineDM_buf[8] = {0};


// 
float sensor_value = 0.0f;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void vesc_decode(){

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8]; // 假设CAN消息数据长度为8字节
	can_recvtimes++;
	
  if (hcan == &hcan1)
  {
    // 从FIFO 0中读取消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      can_1_recvtimes++;
    }
    if((RxHeader.ExtId >> 8) == 0x09)   // 如果是VESC发来的消息
      {
				read_VESC_id = RxHeader.ExtId;
        MotorVescRecvData vesc_recvs; // 新建接收用的结构体
        vesc_recvs.rx_header = RxHeader;

        for (int i = 0; i < 8; i++)
        {
          vesc_recvs.recv_data[i] = RxData[i];
        }
        motor_vesc_handle(vesc_recvs);
      }
    //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }


  else if (hcan == &hcan2)
  {
    // 从FIFO 0中读取消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      can_2_recvtimes++;

      if((RxHeader.ExtId >> 8) == 0x09)   // 如果是VESC发来的消息
      {
				read_VESC_id = RxHeader.ExtId;
        MotorVescRecvData vesc_recvs; // 新建接收用的结构体
        vesc_recvs.rx_header = RxHeader;

        for (int i = 0; i < 8; i++)
        {
          vesc_recvs.recv_data[i] = RxData[i];
        }
        motor_vesc_handle(vesc_recvs);
      }
    }
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
  }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8]; // 假设CAN消息数据长度为8字节
	can_recvtimes++;
	
  if (hcan == &hcan1)
  {
    // 从FIFO 0中读取消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
      can_1_recvtimes++;
    }
    if((RxHeader.ExtId >> 8) == 0x09)   // 如果是VESC发来的消息
      {
				read_VESC_id = RxHeader.ExtId;
        MotorVescRecvData vesc_recvs; // 新建接收用的结构体
        vesc_recvs.rx_header = RxHeader;

        for (int i = 0; i < 8; i++)
        {
          vesc_recvs.recv_data[i] = RxData[i];
        }
        motor_vesc_handle(vesc_recvs);
      }
    //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }


  else if (hcan == &hcan2)
  {
    // 从FIFO 0中读取消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
      can_2_recvtimes++;

      if((RxHeader.ExtId >> 8) == 0x09)   // 如果是VESC发来的消息
      {
				read_VESC_id = RxHeader.ExtId;
        MotorVescRecvData vesc_recvs; // 新建接收用的结构体
        vesc_recvs.rx_header = RxHeader;

        for (int i = 0; i < 8; i++)
        {
          vesc_recvs.recv_data[i] = RxData[i];
        }
        motor_vesc_handle(vesc_recvs);
      }
    }
    //HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
  }
}
uint8_t uart_2_rx_Buffer[48];
uint8_t preserve_Buffer[48];
int uart_recv_size = 0;

uint8_t uart_7_rx_Buffer[24];
uint8_t uart_7_saves_Buffer[24];

uint8_t uart_3_rx_Buffer[20];
uint8_t uart_3_saves_Buffer[20];

int uart_recvs = 0;

int uart_recvs_persec = 0;
int uart_recvs_persecs[16] = {0};
uint8_t uart_recvs_persecs_p = 0;
int all_uart_recvs = 0;

int TimeCounting = 0;
int sieze;

float Byte2Float(uint8_t *byte)
{
    float f;
    uint8_t *p = (uint8_t *)&f;
    p[0] = byte[0];
    p[1] = byte[1];
    p[2] = byte[2];
    p[3] = byte[3];
    return f;
}

//解析上位机的数据
int decode_PC(float *V_x, float *V_y, float *V_w, uint8_t rx_Buffer[])//有问题，不能用
{
  if (rx_Buffer[0] == 0xFA && rx_Buffer[13] == 0xFB)
  {
    float speed[3] = {0.0f, 0.0f, 0.0f};
      // 四字节转浮点数
      for (int i = 0; i < 12; i += 4)
      {
          speed[i / 4] = Byte2Float(rx_Buffer + 1 + i);
          // 想着限幅
      }
      *V_x = speed[0]*100;
      *V_y = speed[1]*100;
      *V_w = speed[2]*100;
			if(*V_x > 5) *V_x = 5;
			if(*V_y > 5) *V_y = 5;
			if(*V_w > 5) *V_w = 5;
    return 1; 
  }
	return 0;
}

/* 串口中断事件回调函数，包含遥控器控制，上位机控制，需在.h中选择模式 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   if (huart == &huart2)   // 来自串口2
   {
			uart_recvs++;
     // 存储到临时存储区
     memcpy(preserve_Buffer, uart_2_rx_Buffer, sizeof(preserve_Buffer));

     // 如果能够成功解码
     if(rmt_decode_joystick_msg(&My_joystick, uart_2_rx_Buffer))
     {
       uart_recvs_persec++;
       // 在成功接收到遥控器时，通知安全守卫
       safe_guard_timer = SAFE_GUARD_TIME;
     }
			
     memset(uart_2_rx_Buffer, 0, sizeof(uart_2_rx_Buffer));
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_2_rx_Buffer, sizeof(uart_2_rx_Buffer));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
   }

   else if (huart == &huart7 && Size == 13)   // 来自串口7
   {
			uart_recvs++;
     // 存储到临时存储区
     memcpy(uart_7_saves_Buffer, uart_7_rx_Buffer, sizeof(uart_7_saves_Buffer));
     decodeVec3(uart_7_saves_Buffer, &chassis_tf);
			
     memset(uart_7_rx_Buffer, 0, sizeof(uart_7_rx_Buffer));
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart_7_rx_Buffer, sizeof(uart_7_rx_Buffer));
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
    }

      
      else if (huart == &huart3)   // 来自串口2
      {
        #ifdef USE_PC
        static float speed[3] = {0.0f, 0.0f, 0.0f};
        uart_recvs++;
        // 存储到临时存储区
        memcpy(uart_3_saves_Buffer, uart_3_rx_Buffer, sizeof(uart_3_saves_Buffer));
        if (uart_3_rx_Buffer[0] == 0xFA && uart_3_rx_Buffer[13] == 0xFB)
        {
          for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(uart_3_rx_Buffer + 1 + i);
                //限幅
                if (speed[i / 4] > 0.8)
                {
                    speed[i / 4] = 0.8;
                }
                else if (speed[i / 4] < -0.8)
                {
                    speed[i / 4] = -0.8;
                }
            }
            Vx = speed[0]*4.5;
            Vy = speed[1]*4.5;  
            Vw = speed[2];
        }
				        uart_recvs_persec++;
        // 在成功接收到遥控器时，通知安全守卫
        safe_guard_timer = SAFE_GUARD_TIME;
        memset(uart_3_rx_Buffer, 0, sizeof(uart_3_rx_Buffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_3_rx_Buffer, sizeof(uart_3_rx_Buffer));
				__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
         #endif
      }
   

}



/* 外部中断回调函数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == muzzle_velotor_0_Pin)    // PA2下降沿触发
    {
      if (current_time - last_pa2_trigger_time >= COOLDOWN_TIME_MS) 
      {
        last_pa2_trigger_time = current_time;
      
        // 停止定时器（如果正在运行），重置计数器并启动
        HAL_TIM_Base_Stop(&htim5);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        HAL_TIM_Base_Start(&htim5);
      }
    }
    else if (GPIO_Pin == muzzle_velotor_1_Pin)   // PA3下降沿触发
    {
      if (current_time - last_pd12_trigger_time >= COOLDOWN_TIME_MS) 
      {
        last_pd12_trigger_time = current_time;
      
        // 停止定时器并读取计数值
        HAL_TIM_Base_Stop(&htim5);
        muzzle_velo_count = __HAL_TIM_GET_COUNTER(&htim5);
        measurement_done = 1;  // 设置标志位
      }
    }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
volatile uint8_t lasdjfkl_flag = 0;
//测试运球的函数，上面为-3000，下面为30000，角度为0.5
void Logic_Yunqiu_test()
{ 
	
	VESC_rpm_up = -3000;
	VESC_rpm_down = 33000;
	  motor_vesc_set_rpm(0, VESC_rpm_up);
    motor_vesc_set_rpm(1, VESC_rpm_up);
    motor_vesc_set_rpm(2, VESC_rpm_down);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(1000);
	
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(10);
	VESC_rpm_up = -3000;
	VESC_rpm_down = -3000;
	  motor_vesc_set_rpm(0, VESC_rpm_up);
    motor_vesc_set_rpm(1, VESC_rpm_up);
    motor_vesc_set_rpm(2, VESC_rpm_down);
	
	lasdjfkl_flag = 0;

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*************    这些是灯带状态的实现变量，全都是全局变量    ****************/


extern Vec3 chassis_speed_vec;
float dzx_freq;
volatile uint8_t eee_flag=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_TIM8_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_UART7_Init();
  MX_TIM7_Init();
  MX_SPI4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /************     初始化 灯带    **************/
  // WS2812_InitBuffer();
  // WS2812_AddStateLink(&carinit_flag, &NJUST_Purple_Expand);
  // WS2812_AddStateLink(&idle_flag, &Test_2c_gradflow);
  // WS2812_AddStateLink(&drbl_flag, &NJUST_Purple_Flow);
  // WS2812_AddStateLink(&gunload_flag, &Wonderful);
  // WS2812_AddStateLink(&FaShe_flag, &Wonderful);
  /************     初始化 气缸GPIO     **************/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);//继电器高电平触发
  
  /******************     初始化 摩擦轮    ************************/
  MotorVESC_Init(&motor_vesc_1, &hcan1, 0, 58);    // 左上
  MotorVESC_Init(&motor_vesc_2, &hcan1, 1, 72);     // 右上
  MotorVESC_Init(&motor_vesc_3, &hcan1, 2, 97);     // 下面那个
  // motor_c620_init(&hcan1);
  // M3508_Left = pids_create_init(0.18, 0.1, 0.0, 0.001, 250, 0.25, 0);
  // M3508_Right = pids_create_init(0.18, 0.1, 0.0, 0.001, 250, 0.25, 0);

  HAL_Delay(100);
  /************     使能定时器中断    **************/
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  HAL_Delay(100);
  /************     初始化CAN1和CAN2    **************/
  bspcan_filter_init_recv_all(&hcan1);
  //CAN_Interrupt_Enable(&hcan1);
  HAL_CAN_Start(&hcan1);
  bspcan_filter_init_recv_all(&hcan2);
//  CAN_Interrupt_Enable(&hcan2);
  HAL_CAN_Start(&hcan2);

  HAL_Delay(100);
  /************     初始化  串口中断    **************/
	//  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE); //不用开
//	__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE); 
//  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE); 
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_2_rx_Buffer, sizeof(uart_2_rx_Buffer));    // 启动 串口DMA 空闲中断
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart_7_rx_Buffer, sizeof(uart_7_rx_Buffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_3_rx_Buffer, sizeof(uart_3_rx_Buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);             // 关闭 DMA 传输过半中断（好像没啥用）
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);             
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT); 	
	

  HAL_Delay(100);
  /************     初始化  达妙电机    **************/
  // DM_buf_send(DM_MOTORID_MAGAZINE, DM_enable_buf, DM_MESTYPE_INIT);
  // HAL_Delay(75);
  // DM_get_posispd_buffer(MagazineDM_buf, 0.0f, DM_TWOROUND_PERSEC);
  // DM_buf_send(DM_MOTORID_MAGAZINE, MagazineDM_buf, DM_MESTYPE_POSITION);
  // HAL_Delay(1000);
  DM_buf_send(DM_MOTORID_SHOOTER, DM_enable_buf, DM_MESTYPE_POSITION);
  HAL_Delay(75);
  DM_get_posispd_buffer(ShooterDM_buf, 0.0f, DM_HALFROUND_PERSEC / 4);
  DM_buf_send(DM_MOTORID_SHOOTER, ShooterDM_buf, DM_MESTYPE_POSITION);

  HAL_Delay(100);
  /****************     方便下代码用的，可以去了    **********************/
  

  //
	DWT_Init(180);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		//测试运球，下面是30000，上面是-3000
		// if(eee_flag==1){
		// 	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		// }
		// else HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		
		// if(lasdjfkl_flag == 1) Logic_Yunqiu_test();
		//测试电机
		// MotorVESC_SetMotorRPM(&motor_vesc_1, motor_vesc_1.motor_rpm_set);
		// MotorVESC_SetMotorRPM(&motor_vesc_2, motor_vesc_2.motor_rpm_set);
		// MotorVESC_SetMotorRPM(&motor_vesc_3, motor_vesc_3.motor_rpm_set);
    // temp_1 = Claw_Dribble_Checker_Up;
    // temp_1 = HAL_GPIO_ReadPin(DribbleConfirm_GPIO_Port, DribbleConfirm_Pin);
    temp_2 = M3508CLAW_DribbleConf;
    // temp_float = magazine_linear(ShooterDM_position);

    
    /***************    向底盘发送消息    *******************/
    static uint32_t dzx_time=0;
    dzx_freq=DWT_GetDeltaT(&dzx_time);
		  // 计算摇杆输入
  rmt_estimate_js_input();
  // 映射摇杆输入到底盘速度
  algo_js_to_chassis_vec(&chassis_speed_vec, My_joystick);
  // 限制底盘输入
  algo_chassis_vec_constrain(&chassis_speed_vec, 0.1);
	


  
  // 右拨杆 确定速度等级
  if (RIGHT_PADDLE_IS_UP)    
  {
    VelocityGain = 1.0;
  }
  else if (RIGHT_PADDLE_IS_MIDDLE)
  {
    VelocityGain = 2.0;
  }
  else if (RIGHT_PADDLE_IS_DOWN)
  {
    VelocityGain = 3.0;
  }
  else
  {
    VelocityGain = 0.5;
  }
  
  // 将底盘速度矢量分解到每个舵轮上
  algo_calc_steer_vecs_3(-Vx * VelocityGain, -Vy * VelocityGain, Vw * RotateVelocityGain, STR_MS);
    
	sendMsgToChassis();

    /***************    计算出膛速度    *******************/
    if (measurement_done)
    {
        measurement_done = 0;   
        // 转换为秒（TIM5时钟为1MHz，每个计数=1μs）
        muzzle_velo_seconds = (float)muzzle_velo_count / 50000.0f;
        // 计算出速度
        muzzle_velo = MUZZLE_CHECKER_LEN / muzzle_velo_seconds;
    }




    /***************    循环控制  达妙  和  VESC，避免丢包    *******************/
    //调整炮塔仰角     (注：数值为RAD，不要填很大！，然后往前倾是负的)
//    DM_get_PosiSpd_bufferlim(ShooterDM_buf, ShooterDM_position, ShooterDM_speed, ShooterDM_Vertical, ShooterDM_Horizontal);
    DM_get_posispd_buffer(ShooterDM_buf, ShooterDM_position, ShooterDM_speed); 
		DM_buf_send(DM_MOTORID_SHOOTER, ShooterDM_buf, DM_MESTYPE_POSITION);
    HAL_Delay(1);
    // // 调整弹匣开合
    // DM_get_PosiSpd_bufferlim(MagazineDM_buf, MagazineDM_position, MagazineDM_speed, 0.06, magazine_linear(ShooterDM_position));
    // DM_buf_send(DM_MOTORID_MAGAZINE, MagazineDM_buf, DM_MESTYPE_POSITION);

    motor_vesc_set_rpm(0, VESC_rpm_up);
    motor_vesc_set_rpm(1, VESC_rpm_up);
    motor_vesc_set_rpm(2, VESC_rpm_down);

    /***************    发送串口消息    *******************/
    //snprintf((char*)vofa_data, sizeof(vofa_data), "d:%.2f,%.2f\n", m3508L_max_current, m3508R_max_current);
    // HAL_UART_Transmit_DMA(&huart6, vofa_data, sizeof(vofa_data));

    HAL_Delay(9);
    TimeCounting += 10;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 编码函数实现
void encodeVec3(const Vec3* vec, uint8_t* buffer) {
    memcpy(buffer, &vec->x, sizeof(float));
    memcpy(buffer + sizeof(float), &vec->y, sizeof(float));
    memcpy(buffer + 2 * sizeof(float), &vec->z, sizeof(float));
}

// 解码函数实现
void decodeVec3(const uint8_t* buffer, Vec3* vec) {
    memcpy(&vec->x, buffer, sizeof(float));
    memcpy(&vec->y, buffer + sizeof(float), sizeof(float));
    memcpy(&vec->z, buffer + 2 * sizeof(float), sizeof(float));
}


void CAN_Interrupt_Enable(CAN_HandleTypeDef *can_n)
{
    if (HAL_CAN_ActivateNotification(can_n, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
        // 中断配置失败处理
    }
}

// 安全锁 置位 时，才可向底盘发送消息
float	steer_angle_offset_0 = 0;
float	steer_angle_offset_1	= 0;
float	steer_angle_offset_2 = 0;//舵轮零位补偿，该值大于0时，轮顺时针转
void sendMsgToChassis()
{
  if (safe_locks)
    {
        if (STR_MS[0].x < 120)   // 如果慢于这个速度
        {
          sendCanTXBuffer(0, algo_steerangles_preserve[0], 1);
        }
        else
        {
          algo_steerangles_preserve[0] = STR_MS[0].y;   // 存储下角度
					algo_steerangles_preserve[0] += steer_angle_offset_0;
          sendCanTXBuffer(STR_MS[0].x * VelocityGain, algo_steerangles_preserve[0], 1);
        }

        if (STR_MS[1].x < 120)   // 如果慢于这个速度
        {
          sendCanTXBuffer(0, algo_steerangles_preserve[1], 2);
        }
        else
        {
          algo_steerangles_preserve[1] = STR_MS[1].y;   // 存储下角度
					algo_steerangles_preserve[1] += steer_angle_offset_1;
          sendCanTXBuffer(STR_MS[1].x * VelocityGain, algo_steerangles_preserve[1], 2);
        }

        if (STR_MS[2].x < 120)   // 如果慢于这个速度
        {
          sendCanTXBuffer(0, algo_steerangles_preserve[2], 3);
        }
        else
        {
          algo_steerangles_preserve[2] = STR_MS[2].y;   // 存储下角度
					algo_steerangles_preserve[2] += steer_angle_offset_2;
          sendCanTXBuffer(STR_MS[2].x * VelocityGain, algo_steerangles_preserve[2], 3);
        }
    }
    else    // 安全锁不置位时，确认链接
    {
        sendCanTXBuffer_Confirm(1);
        sendCanTXBuffer_Confirm(2);
        sendCanTXBuffer_Confirm(3);
    }
}






void sendCanTXBuffer(int speed_rpm, float steer_angle_deg, uint8_t Steer_ID)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = 0x08 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_EXT ;		          // 发送的帧是扩展帧
  TxMsg.ExtId = STEER_Control * Steer_ID;             // CAN_ID

  TxMsg.TransmitGlobalTime = DISABLE; 

  // 调制速度信息
  sendbuf[0] = speed_rpm >> 24;
  sendbuf[1] = speed_rpm >> 16;
  sendbuf[2] = speed_rpm >> 8;
  sendbuf[3] = speed_rpm & 0xff;

  // 调制角度信息（将0-360度变换为0-16384）
  // 角度归一化到0-360度
  while (steer_angle_deg < 0) steer_angle_deg += 360.0;
  while (steer_angle_deg >= 360.0) steer_angle_deg -= 360.0;

  // 转换为0-16384范围内的整数
  int steer_angle_code = (int)((steer_angle_deg / 360.0) * 16384);
  sendbuf[4] = steer_angle_code >> 24;
  sendbuf[5] = steer_angle_code >> 16;
  sendbuf[6] = steer_angle_code >> 8;
  sendbuf[7] = steer_angle_code & 0x00ff;

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);     // 等待CAN邮箱

	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, sendbuf, &txmailbox);
}





/**
 * 
 * 
 * 
 */
void sendCanTXBuffer_Confirm(uint8_t Steer_ID)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = (uint8_t)8 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_EXT ;		          // 发送的帧是扩展帧
  TxMsg.ExtId = STEER_LinkConfirm * Steer_ID;             // CAN_ID
  TxMsg.TransmitGlobalTime = DISABLE; 

  sendbuf[0] = 0;
  sendbuf[1] = 0;
  sendbuf[2] = 0;
  sendbuf[3] = 0;
  sendbuf[4] = 0;
  sendbuf[5] = 0;
  sendbuf[6] = 0;
  sendbuf[7] = 0;

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);     // 等待CAN邮箱
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, sendbuf, &txmailbox);

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
