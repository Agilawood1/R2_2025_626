#ifndef _AD7606_H
#define _AD7606_H

#include "stdarg.h"


#define rst_Set 	 		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)  // PC4   
#define rst_Reset  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)

#define convstB_Set 	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)        // PC6
#define convstB_Reset   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)

#define convstA_Set 	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define convstA_Reset   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

#define STby_Set 	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)        //  PC1
#define STby_Reset 	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)

#define OS12_Set 	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)        // PA3
#define OS12_Reset       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)

#define OS11_Set 	      HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET)        // PI5
#define OS11_Reset       HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET)

#define OS10_Set 	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define OS10_Reset       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)           // PB0


#define frstdata_Set 	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)            // PD12
#define frstdata_Reset  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)

#define busy_Set 	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)          // PD13
#define busy_Reset   		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)

#define cs_Set 	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)         // PD14
#define cs_Reset   			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)

#define clk_Set 	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)         // PD15
#define clk_Reset   		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)

#define read_busy       HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)          //PD13

extern void GPIO_AD7606_Configuration(void);
extern void AD7606_InitTest(void);
extern void AD7606_startconvst(void);
extern void AD7606_reset(void) ;
extern void AD7606_read_data(int16_t *DB_data);
extern void AD7606_Init(void);

extern void AD7606_ReadData(int16_t *Targ_data);
extern void AD7606_ReadData_Percent(float *Targ_data);

extern int16_t DB_data_save[8];
extern float SickDistance[8];
extern int16_t Receive_data[8];
#endif
