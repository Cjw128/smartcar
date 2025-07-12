// param.c
#include "zf_common_headfile.h"
#include "param.h"

ParamStruct params = {0.25f, 30.0f, 0.5f};  // 默认值

#define FLASH_ADDR  (0x0803F800)  // 根据你芯片实际Flash地址调整，逐飞推荐最后一页

void param_flash_write(void)
{
    flash_erase_sector(FLASH_ADDR);  // 擦除扇区
    flash_write_bytes(FLASH_ADDR, (uint8 *)&params, sizeof(ParamStruct));
}

void param_flash_read(void)
{
    flash_read_bytes(FLASH_ADDR, (uint8 *)&params, sizeof(ParamStruct));
}