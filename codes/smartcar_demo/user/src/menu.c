#include "zf_driver_flash.h"
#include "zf_common_headfile.h"
#include "motor.h"
#include "menu.h"

int menu1(void)
{
    int k1 = 0, k2 = 0, k3 = 0;
    int flag = 1;

    // 初始化显示
    ips200_clear();
    ips200_show_string(0, 100, "Menu 1: test1");
    ips200_show_string(0, 120, "Menu 2: test2");
    ips200_show_string(0, 140, "Menu 3: test3");

    while (1)
    {
        key_scanner();

        key_state_enum state1 = key_get_state(KEY_1);
        key_state_enum state2 = key_get_state(KEY_2);
        key_state_enum state3 = key_get_state(KEY_3);

        if (state1 == KEY_SHORT_PRESS) k1 = 1;
        if (state2 == KEY_SHORT_PRESS) k2 = 1;
        if (state3 == KEY_SHORT_PRESS) k3 = 1;

        if (k2)
        {
            flag++;
            if (flag > 3) flag = 1;
            k2 = 0;
        }
        if (k1)
        {
            flag--;
            if (flag < 1) flag = 3;
            k1 = 0;
        }
        if (k3)
        {
            system_delay_ms(10);
    // 等待按键释放，同时持续扫描按键状态
    while (key_get_state(KEY_3) != KEY_RELEASE)
    {
        key_scanner();  // 不断刷新按键状态
        system_delay_ms(10);
    }
    system_delay_ms(10);
    return flag;
        }

        // 清除选择标志
        ips200_show_string(120, 100, " ");
        ips200_show_string(120, 120, " ");
        ips200_show_string(120, 140, " ");

        // 显示当前选择
        ips200_show_string(120, 100+(flag - 1) * 20, "x");

        system_delay_ms(100);
    }
}

int menu2_1(void)
{
    ips200_clear();
    
    while (1)
    {
      image_process();
			
			
    }
}
int menu2_2(void)
{

		ips200_clear();
    ips200_show_string(0, 240, "Press KEY_3 to confirm");
    ips200_show_string(0, 260, "Press KEY_4 to go back");
    int k3 = 0;
    int k4 = 0;
    while (1)
    {
			
			key_scanner();
				
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
            k3 = 1;
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
            k4 = 1;
			

        if (k3)
        {
            system_delay_ms(10);
            while (key_get_state(KEY_3) != KEY_RELEASE)
            {
                key_scanner();
                system_delay_ms(10);
            }
            system_delay_ms(10);
            return 1;  // 比如确认返回1
        }

        // 按KEY4返回上一级菜单
        if (k4)
        {
            system_delay_ms(10);
            while (key_get_state(KEY_4) != KEY_RELEASE)
            {
                key_scanner();
                system_delay_ms(10);
            }
            system_delay_ms(10);
            return 0;  // 返回0表示返回上一级
        }


    }
}
#define PARAM_FLASH_SECTOR 127   // 你选择的扇区号，范围一般是0-127
#define PARAM_FLASH_PAGE   3     // 你选择的页码，范围一般是0-3

// 参数变量
param_config_t params = {
    .Kp_dir = 0.25f,
    .Kp_slope = 40.0f,
    .Kd_dir = 0.4f,
    .base_speed = 38.0f,
    .target_speed = 40.0f
};

#define PARAM_FLASH_WORD_LEN 5  // 一共 5 个 float 参数

void param_flash_write(void)
{
    if (flash_erase_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE) != 0)
    {
        printf("Flash erase failed!\r\n");
        return;
    }

    if (flash_write_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE, (const uint32 *)&params, PARAM_FLASH_WORD_LEN) != 0)
    {
        printf("Flash write failed!\r\n");
        return;
    }

    printf("参数已成功保存到 Flash！\r\n");
}

void param_flash_read(void)
{
    flash_read_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE, (uint32 *)&params, PARAM_FLASH_WORD_LEN);
}
// 参数枚举，方便管理

typedef enum
{
    PARAM_KP_DIR = 0,
    PARAM_KP_SLOPE,
    PARAM_KD_DIR,
    PARAM_BASE_SPEED,
    PARAM_TARGET_SPEED,
    PARAM_TOTAL          // 总数，必须放在最后
} param_index_t;

int menu_param_config(void)
{
    int selected = 0;
    int k1 = 0, k2 = 0, k3 = 0, k4 = 0;

    while (1)
    {
        ips200_clear();

        // 显示各参数
        ips200_show_string(0, 100,  "Kp_dir       :");
        ips200_show_float(100, 100,  params.Kp_dir, 5, 2);

        ips200_show_string(0, 120, "Kp_slope     :");
        ips200_show_float(100, 120, params.Kp_slope, 5, 2);

        ips200_show_string(0, 140, "Kd_dir       :");
        ips200_show_float(100, 140, params.Kd_dir, 5, 2);

        ips200_show_string(0, 160, "base_speed   :");
        ips200_show_float(100, 160, params.base_speed, 5, 1);

        ips200_show_string(0, 180, "target_speed :");
        ips200_show_float(100, 180, params.target_speed, 5, 1);

        // 清除箭头区并绘制当前箭头
        for (int i = 0; i < PARAM_TOTAL; i++)
            ips200_show_string(180, 100 + i * 20, " ");
        ips200_show_string(180, 100 + selected * 20, "<");

        // 扫描按键
        key_scanner();
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) k1 = 1;
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) k2 = 1;
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) k3 = 1;
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) k4 = 1;

        // KEY2 下移选择
        if (k2)
        {
            selected++;
            if (selected >= PARAM_TOTAL) selected = 0;
            k2 = 0;
        }

        // KEY3 增加参数
        if (k3)
        {
            switch (selected)
            {
                case PARAM_KP_DIR:        params.Kp_dir        += 0.05f; break;
                case PARAM_KP_SLOPE:      params.Kp_slope      += 1.0f;  break;
                case PARAM_KD_DIR:        params.Kd_dir        += 0.05f; break;
                case PARAM_BASE_SPEED:   params.base_speed    += 1.0f;  break;
                case PARAM_TARGET_SPEED: params.target_speed  += 1.0f;  break;
            }
            k3 = 0;
        }

        // KEY1 减少参数
        if (k1)
        {
            switch (selected)
            {
                case PARAM_KP_DIR:        params.Kp_dir        -= 0.05f; break;
                case PARAM_KP_SLOPE:      params.Kp_slope      -= 1.0f;  break;
                case PARAM_KD_DIR:        params.Kd_dir        -= 0.05f; break;
                case PARAM_BASE_SPEED:   params.base_speed    -= 1.0f;  break;
                case PARAM_TARGET_SPEED: params.target_speed  -= 1.0f;  break;
            }
            k1 = 0;
        }

        // KEY4 保存并退出
        if (k4)
        {
            while (key_get_state(KEY_4) != KEY_RELEASE) key_scanner();
            param_flash_write();  // 写入Flash
            return 0;             // 返回上一级菜单
        }

        system_delay_ms(100);
    }
}
