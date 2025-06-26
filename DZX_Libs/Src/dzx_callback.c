#include "main.h"
#include "usart.h"
#include "can.h"
#include "dzx_lib.h"
#include "bsp_can.h"
#include "string.h"
extern uint8_t uart_2_rx_Buffer[48];
extern uint8_t uart_3_rx_Buffer[20];
extern uint8_t uart_3_saves_Buffer[20];
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	__HAL_UNLOCK(huart);
	if(huart==&huart2){
		memset(uart_2_rx_Buffer, 0, sizeof(uart_2_rx_Buffer));
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_2_rx_Buffer, sizeof(uart_2_rx_Buffer));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
	else if(huart==&huart3){
		memset(uart_3_rx_Buffer, 0, sizeof(uart_3_rx_Buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_3_rx_Buffer, sizeof(uart_3_saves_Buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t errorcode = hcan->ErrorCode;
    
    if (hcan->Instance == CAN1) {
        // CAN1������
        if (errorcode & HAL_CAN_ERROR_TX_ALST0) {
            // �ٲö�ʧ����
            HAL_CAN_ResetError(hcan);
        }
        if (errorcode & HAL_CAN_ERROR_TIMEOUT) {
            // ��ʱ����
            HAL_CAN_Start(hcan);
        }
        if (errorcode & HAL_CAN_ERROR_BOF) {
            // ���߹رմ���
            HAL_CAN_Stop(hcan); 
						bspcan_filter_init_recv_all(&hcan1); // �������ù�����
            HAL_CAN_Start(hcan);
           
        }                          
    }
    else if (hcan->Instance == CAN2) {
        // CAN2������
        if (errorcode & HAL_CAN_ERROR_TX_ALST0) {
            // �ٲö�ʧ����
            HAL_CAN_ResetError(hcan);
        }
        if (errorcode & HAL_CAN_ERROR_TIMEOUT) {
            // ��ʱ����
            HAL_CAN_Start(hcan);
        }
        if (errorcode & HAL_CAN_ERROR_BOF) {
            // ���߹رմ���
            HAL_CAN_Stop(hcan);
            bspcan_filter_init_recv_all(&hcan2);  // �������ù�����
            HAL_CAN_Start(hcan);
            
        }                                 
    }
}










