#include "main.h"
#include "AD7606.h"
#include "delay.h"
//-----------------------------------------------------------------
// ��ʼ��������
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// void GPIO_AD7606_Configuration(void)
//-----------------------------------------------------------------
//
// ��������: AD7606�������ú���
// ��ڲ���?: ��
// ���ز���: ��
// ȫ�ֱ���: ��
// ����ģ��: __HAL_RCC_GPIOA_CLK_ENABLE(); HAL_GPIO_DeInit();
// ע������: ��GPIO������ʽ��FSMC������ʽ�µ��������ò�һ��
//			
//-----------------------------------------------------------------
/*
	�밴���·�ʽ��������
	ʹ�����ţ�
	RST: PC4
	ConvstA: PC0
	ConvstB: PC5
	STby: PC1

	OSI2: PA3
	OSI1: PI5   
	OSI0: PB0

	frstdata: PD12
	busy: PD13
	cs: PD14
	rd: PD15

	DB7: PH12
*/
int16_t DB_data_save[8] = {0};
float SickDistance[8]={0};
int16_t Receive_data[8] = {0};

void AD7606_Init(void)		// AD7606�����ʼ��?
{
	HAL_Delay(500);
	// delay_ms(500);
	GPIO_AD7606_Configuration();
	AD7606_InitTest();
	// delay_ms(500);
	HAL_Delay(500);
}


/*
	函数名：读取数值
	输入值：用于接收消息的数组
	返回值：无
	作用：将数值写入传入的数组，并且每次调用都会更新DB_data_save数组
*/
void AD7606_ReadData(int16_t *Targ_data)
{
	AD7606_startconvst();
		
	int State_busy = read_busy; // 读取 BUSY的状态
	
	while ((State_busy == 1))  // 当busy为低电平时，数据转换完毕，此时可以读取数据，等于高就继续读取
	{
		delay_100ns();
		State_busy = read_busy; // 读取 BUSY的状态
	}
	
	delay_us(1);
	AD7606_read_data(Targ_data); // 读取数据放至数组DB_data[]
}

/*
	函数名：读取数值
	输入值：用于接收消息的数组
	返回值：无
	作用：将数值写入传入的数组，并且每次调用都会更新DB_data_save数组
	
	*传回的值是一个百分比，数值为 Num / 32768
*/
void AD7606_ReadData_Percent(float *Targ_data)
{
	AD7606_startconvst();
		
	int State_busy = read_busy; // ��ȡ BUSY��״̬
	
	while ((State_busy == 1)) // ��busyΪ�͵�ƽʱ������ת����ϣ���ʱ���Զ�ȡ���ݣ����ڸ߾ͼ������?
	{
		HAL_Delay(1);
		State_busy = read_busy; // ��ȡ BUSY��״̬
	}
	
	HAL_Delay(1);
	int16_t temp_data[8] = {0};
	AD7606_read_data(temp_data); // ��ȡ����
	Targ_data[0] = temp_data[0] / 32768.0;
		Targ_data[0] *= 5769.23;
////	Targ_data[0]+=100;
	Targ_data[1] = temp_data[1] / 32768.0;
		Targ_data[1] *= 5540.12;
//	for (int i = 0; i < 8; i++)
//	{
//		Targ_data[i] = temp_data[i] / 32768.0;
//		Targ_data[i]*=5769.23;
////		Targ_data[1]*=5514.71;
//	}
}

void GPIO_AD7606_Configuration(void)		// AD7076��ʼ��
{ 
	GPIO_InitTypeDef GPIO_InitStructure;

	// ʹ��IO��ʱ��
	  __HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOI_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();
					
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);			// rst			//	��GPIOx����Ĵ�����ʼ���?Ĭ�ϸ�λֵ
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);			// ConvstA
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);			// ConvstB
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);			// STby

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);			// OSI2
	HAL_GPIO_DeInit(GPIOI, GPIO_PIN_5);			// OSI1
//	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);			// OSI1
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);			// OSI0

	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);		// frstdata
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_13);		// busy
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_14);		// cs
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_15);		// rd

	HAL_GPIO_DeInit(GPIOH, GPIO_PIN_12);		// DB7
//	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);		// DB7



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// AD7606 
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//���������� 
	//            CS_N       RD/SCLK      
			GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_15;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//            FRSTDATA     BUSY  
			GPIO_InitStructure.Pin = GPIO_PIN_12 | GPIO_PIN_13;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_EVT_RISING_FALLING;
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//    rst convstB convstA STby
			// GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_10 | GPIO_PIN_9;
			GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_4  ;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//	   OS12 OS11 OS10
			GPIO_InitStructure.Pin = GPIO_PIN_3;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_InitStructure.Pin = GPIO_PIN_5;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);

			GPIO_InitStructure.Pin = GPIO_PIN_0;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  //������(1)
	// DoutA
	    GPIO_InitStructure.Pin = GPIO_PIN_12 ;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
			
			
			GPIO_InitStructure.Pin = GPIO_PIN_0;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);
}

//-----------------------------------------------------------------
// void AD7606_Init(void)
//-----------------------------------------------------------------
//
// 函数功能: AD7606初始化函数
// 入口参数: 无
// 返回参数: 无
// 全局变量: 无
// 调用模块:    
// 注意事项: 无
//-----------------------------------------------------------------
void AD7606_InitTest(void)
{
	convstA_Set;
	convstB_Set;
	STby_Set;
	clk_Set;
	cs_Set;	
  OS10_Reset;
	OS11_Reset;
	OS12_Reset;
	AD7606_reset();  
	AD7606_startconvst();	
}

/*   * 名称：AD7606_startconvst()  * 功能：启动转换  */  
void AD7606_startconvst(void)
{  
	convstA_Reset;	
	convstB_Reset;	
	delay_us(11);
	convstA_Set;
	convstB_Set;
}
  
/*   * 名称：AD7606_reset()  * 功能：复位模块  */
void AD7606_reset(void) 
{ 
	rst_Reset;
	delay_100ns();
	rst_Set; 
	delay_us(1);
	rst_Reset; 
}  
/* 
* 名称：AD7606_read_data() 
* 功能：读取数据 
* 返回值：返回一个结构体指针，该指针为指向结构体数组的首地址  
*/ 
void AD7606_read_data(int16_t *DB_data) 
{  
	uint8_t i,j; 	
	for(i=0;i<8;i++)  
	{
		uint16_t DB_data1 = 0;
		cs_Reset; 
	  delay_us(3);

		for(j=0;j<16;j++)
		{		
		clk_Reset;
		delay_us(4);
		DB_data1 = ((uint16_t)(HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12))<<(15-j)) + DB_data1;

	  clk_Set;
		delay_us(4);		
		}		
		cs_Set;	
		DB_data[i] = (uint16_t)DB_data1;
		DB_data_save[i] = (uint16_t)DB_data1;
	}	
	
} 
