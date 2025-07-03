#include "zf_common_headfile.h"
#include "menu.h"
#include "otsu.h"
int menu1(void)
{
    int k1 = 0, k2 = 0, k3 = 0;
    int flag = 1;

    // 初始化显示
    ips200_clear();
    ips200_show_string(0, 0, "Menu 1: test1");
    ips200_show_string(0, 20, "Menu 2: test2");
    ips200_show_string(0, 40, "Menu 3: test3");

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
        ips200_show_string(120, 0, " ");
        ips200_show_string(120, 20, " ");
        ips200_show_string(120, 40, " ");

        // 显示当前选择
        ips200_show_string(120, (flag - 1) * 20, "x");

        system_delay_ms(100);
    }
}

int menu2_1(void)
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
		if (mt9v03x_finish_flag)
        {
            ips200_display_graph();
            mt9v03x_finish_flag = 0;
        }
        // 按KEY3确认操作，演示用，直接返回1（或你自定义）
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

        system_delay_ms(100);
    }
}

uint8 image1[MT9V03X_W * MT9V03X_H];
void ips200_display_graph()
{
    memcpy(image1, mt9v03x_image, MT9V03X_W * MT9V03X_H);
    highlight_processing((uint8*)image1,MT9V03X_W,MT9V03X_H,235);
    uint8 threshold = otsu_find_threshold((uint8*)image1,MT9V03X_W,MT9V03X_H);
    ips200_show_gray_image(0, 0, (const uint8 *)image1, MT9V03X_W, MT9V03X_H, 240, 180, threshold);
}