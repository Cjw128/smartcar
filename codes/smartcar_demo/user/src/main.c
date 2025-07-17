/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

    #include "zf_common_headfile.h"
    #define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 双排排针 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
    //#define ENABLE_IMAGE_DISPLAY 1  // 改成 1 就可以重新开启图像显示                                                                                // 单排排针 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

     #include "menu.h"
		 #include "motor.h"
		 #include "isr.h"
    volatile uint8 image_ready_flag = 0;

// **************************** 代码区域 ****************************

enum MenuState
{
    MENU_MAIN,
    MENU_2_1,
    MENU_2_2,
    MENU_2_3,
};

int main(void)
{
    
	clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
	debug_init();                                                               // 初始化默认 Debug UART
    key_init(10);
	mpu6050_init();
    mt9v03x_init();
	motor_init();
    encoder_init();
	pid_speed_init_from_params();
  param_flash_read();  // 从Flash加载上次保存的参数
		ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_init(IPS200_TYPE_SPI);
		extern param_config_t params;

    pit_ms_init(TIM6_PIT, 2);   	// 初始化 5ms 周期定时器
    int current_menu = 1;  // 主菜单或初始菜单编号
    int ret;

    	
    while (1)
    {		
			
         switch (current_menu)
        {
            case 1:
                // 这里调用普通菜单1
                current_menu = menu1();  // 你的已有菜单函数
                break;

            case 2:
                // 调用参数调节菜单
                ret = menu_param_config();
                // menu_param_config返回0时表示退出到主菜单
                if (ret == 0)
                {
                    current_menu = 1;  // 返回主菜单
                }
                break;

            // 其它菜单
            case 3 :
                ret = menu2_1();
						
					
					
                break;
        }


    

			
    }


	}