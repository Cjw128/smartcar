#include "zf_common_headfile.h"
#include "motor.h"
#include "otsu.h"
#include <math.h>
#include "menu.h"
/*****************************************编码器部分**********************************************/
#define ENCODER_1                   (TIM3_ENCODER)
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)

#define ENCODER_2                   (TIM4_ENCODER)
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

#define PIT                         (TIM6_PIT )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
/*****************************************编码器部分**********************************************/
extern param_config_t params;  // 从菜单模块获取最新参数

/******************************************电机部分**********************************************/
#define MAX_DUTY            (40)                                             // 最大 MAX_DUTY% 占空比
#define DIR_L               (A0 )
#define PWM_L               (TIM5_PWM_CH2_A1)

#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH4_A3)
/******************************************电机部分**********************************************/
//int16 target_speed_L = 40;  // 左轮目标速度
//int16 target_speed_R = 40;  // 右轮目标速度
												// 目标速度，单位：编码器计数 / 100ms（可理解为“速度”）
//float Kp = 0.5f;                     // 比例系数，可调整
int8 pwm_duty = 0;                  // 当前占空比（-50 ~ +50）

//float Kp_dir = 0.30f;        // 比例系数（调整方向控制灵敏度）
//float Kd_dir = 0.34f;        // 微分系数（控制震荡）
//float Kp_slope = 40.0f; 
//int16 base_speed = 38;      // 基础速度（根据调试调整）

int16 last_deviation = 0;   // 上一次偏差
int16 last_slope = 0;

int16 limit_a_b(int16 x, int a, int b);
extern float Kp_dir, Kp_slope, Kd_dir;

void motor_init(void)
{
    // 初始化左电机方向引脚
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 初始化左电机 PWM（10KHz，初始占空比 0）
    pwm_init(PWM_L, 10000, 0);

    // 初始化右电机方向引脚
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 初始化右电机 PWM（10KHz，初始占空比 0）
    pwm_init(PWM_R, 10000, 0);
}
void encoder_init(void)
{
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);  // 编码器1初始化
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);  // 编码器2初始化
}



void set_motor_independent(int duty_L, int duty_R)
{
    duty_L = limit_a_b(duty_L, -MAX_DUTY, MAX_DUTY);
    duty_R = limit_a_b(duty_R, -MAX_DUTY, MAX_DUTY);

    // 左电机
    if (duty_L >= 0)
    {
        gpio_set_level(DIR_L, GPIO_LOW);
        pwm_set_duty(PWM_L, duty_L * 4000 / 100);
    }
    else
    {
        gpio_set_level(DIR_L, GPIO_HIGH);
        pwm_set_duty(PWM_L, -duty_L * 4000 / 100);
    }

    // 右电机
    if (duty_R >= 0)
    {
        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, duty_R * 4000 / 100);
    }
    else
    {
        gpio_set_level(DIR_R, GPIO_HIGH);
        pwm_set_duty(PWM_R, -duty_R * 4000 / 100);
    }
}
extern uint8 center_line[image_h];  // 中线数组，图像处理生成
extern void set_motor_independent(int duty_L, int duty_R);  // 你已有的电机控制函数

// 限幅函数（防止速度超出范围）
int16 limit_pwm(int16 x, int limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}
#define FIT_POINT_NUM 8
#define FIT_START_ROW (image_h - 16)   // 稍微提前
#define FIT_ROW_STEP 2                // 跨行大一点（拉长趋势线）
float get_center_slope(void)
{
    int i, count = 0;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_yy = 0;

    for (i = 0; i < FIT_POINT_NUM; i++)
    {
        int y = FIT_START_ROW - i * FIT_ROW_STEP;
        if (y < 0 || y >= image_h) continue;  // 防越界

        int x = center_line[y];
        if (x <= 5 || x >= image_w - 5) continue;  // 过滤无效点

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_yy += y * y;
        count++;
    }

    if (count < 3) return 0;  // 有效点太少，认为趋势无偏斜

    float k = (count * sum_xy - sum_x * sum_y) / (count * sum_yy - sum_y * sum_y + 1e-6f);  // 避免除 0
    return k;
}
// 巡线闭环控制主函数（定时器中调用）
#define SLOPE_GAIN 2.0f
void line_follow_control(void)
{		
	 if (loss_track(mt9v03x_image))  // 图像失踪，立即保护
    {
        set_motor_independent(0, 0);  // 停车
        return;
    }
    
int current_row = image_h - 40;
int16 center = center_line[current_row];
int16 deviation = center - (image_w / 2);
int16 diff = deviation - last_deviation;
last_deviation = deviation;

// 趋势斜率
float slope = get_center_slope();

// 组合PD控制：横向偏差 + 趋势修正 + 微分
float pid_output =
    params.Kp_dir * deviation
  - params.Kp_slope * (SLOPE_GAIN * slope + SLOPE_GAIN * slope * fabsf(slope))
  + params.Kd_dir * diff;
int16 speed_L = limit_pwm(params.base_speed + pid_output, MAX_DUTY);
int16 speed_R = limit_pwm(params.base_speed - pid_output, MAX_DUTY);

set_motor_independent(speed_L, speed_R);
}


void pit_handler(void)
{static uint8 frame_counter = 0;

    
		line_follow_control();
}