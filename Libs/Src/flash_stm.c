/* flash_lib.c */
#include "flash_stm.h"
#include <string.h>

static uint32_t flash_initialized = 0;
static uint32_t flash_addr = USER_FLASH_START_ADDR;


/**
 * @brief  初始化Flash存储模块并验证已有数据
 * @note   该函数会检查存储地址的初始值，判断是否已有有效数据写入。必须在所有其他Flash操作函数前调用
 * @return Flash操作状态:- FLASH_OK: 初始化成功完成
 */
Flash_Status Flash_Init(void) {
    // 验证是否已有数据写入（简单校验）
    uint32_t data = *(__IO uint32_t*)flash_addr;
    flash_initialized = (data != 0xFFFFFFFF);
    return FLASH_OK;
}

/**
 * @brief  将浮点数值写入Flash存储器
 * @param  data: 要存储的浮点数值，范围需符合IEEE754单精度格式
 * @warning 该操作将擦除整个扇区数据，频繁使用将缩短Flash寿命
 * @note   操作流程：
 *         1. 解锁Flash
 *         2. 擦除目标扇区
 *         3. 写入数据
 *         4. 重新锁定Flash
 * @return Flash操作状态:
 *         - FLASH_OK: 写入成功
 *         - FLASH_ERASE_ERROR: 扇区擦除失败
 *         - FLASH_WRITE_ERROR: 数据写入失败
*/
Flash_Status Flash_WriteFloat(float data) {
    Flash_Status status = FLASH_OK;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error;
    
    // 转换为uint32_t进行存储
    uint32_t data_raw = 0;
    memcpy(&data_raw, &data, sizeof(data));

    HAL_FLASH_Unlock();

    // 配置擦除参数
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = USER_FLASH_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    // 擦除扇区
    if(HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        status = FLASH_ERASE_ERROR;
        goto exit;
    }

    // 写入数据
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, data_raw) != HAL_OK) {
        status = FLASH_WRITE_ERROR;
        goto exit;
    }

    flash_initialized = 1;

exit:
    HAL_FLASH_Lock();
    return status;
}


/**
 * @brief  从Flash存储器读取浮点数值
 * @param  data: 输出参数，指向要接收读取结果的浮点数指针
 * @note   读取前需要确保：
 *         - 已成功调用Flash_Init()
 *         - 存储器已有有效数据（非0xFFFFFFFF）
 * @return Flash操作状态:
 *         - FLASH_OK: 读取成功
 *         - FLASH_NOT_INITIALIZED: 未初始化Flash模块
 *         - FLASH_READ_ERROR: 读取到无效数据
*/
Flash_Status Flash_ReadFloat(float *data) {
    if(!flash_initialized) {
        return FLASH_NOT_INITIALIZED;
    }

    uint32_t data_raw = *(__IO uint32_t*)flash_addr;
    
    if(data_raw == 0xFFFFFFFF) {
        return FLASH_READ_ERROR;
    }

    memcpy(data, &data_raw, sizeof(*data));
    return FLASH_OK;
}