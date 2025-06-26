#ifndef FLASH_STM_H
#define FLASH_STM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

// 使用最后一个扇区（根据数据手册调整）
#define USER_FLASH_START_ADDR  0x081C0000  // 扇区11起始地址
#define USER_FLASH_SECTOR      FLASH_SECTOR_11
#define USER_FLASH_SIZE        4           // 存储float占4字节
#define FLASH_VOLTAGE_RANGE_3  0        // Flash编程电压范围,3表示电压范围2.7V-3.6V

typedef enum {
    FLASH_OK,
    FLASH_ERASE_ERROR,
    FLASH_WRITE_ERROR,
    FLASH_READ_ERROR,
    FLASH_NOT_INITIALIZED
} Flash_Status;

Flash_Status Flash_Init(void);
Flash_Status Flash_WriteFloat(float data);
Flash_Status Flash_ReadFloat(float *data);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_LIB_H */