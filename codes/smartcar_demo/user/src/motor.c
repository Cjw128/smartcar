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
#define MAX_DUTY            (50)                                             // 最大 MAX_DUTY% 占空比
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

#define OFFSET -6.524390
// 定义静态变量用于滤波状态
static float gyro_z_filtered = 0.0f;

// 滤波系数（0~1，越大越快，越小越稳，建议 0.1~0.3）
#define GYRO_FILTER_ALPHA  0.15f

// 角速度滤波函数（返回滤波后的结果）
float get_filtered_gyro_z(float raw_gyro_z_dps)
{
    gyro_z_filtered = (1.0f - GYRO_FILTER_ALPHA) * gyro_z_filtered
                    + GYRO_FILTER_ALPHA * raw_gyro_z_dps;
    return gyro_z_filtered;
}
// 巡线闭环控制主函数（定时器中调用）
float center_weighted_deviation(uint8 start, uint8 end)
{
    if (end < start)
    {
        uint8 temp = start;
        start = end;
        end = temp;
    }

    if (start >= image_h) start = image_h - 1;
    if (end >= image_h) end = image_h - 1;

    float sum = 0;
    float weight_sum = 0;

    for (int i = start; i <= end; i++)
    {
        uint8 c = center_line[i];
        if (c > 5 && c < image_w - 5)  // 过滤异常值
        {
            float weight = i - start + 1;  // 行号越靠近底部，权重越大（例如从1开始累加）
            float offset = (image_w / 2.0f) - c;

            sum += offset * weight;
            weight_sum += weight;
        }
    }

    if (weight_sum > 0)
        return sum / weight_sum;
    else
        return 0;  // 全部无效则偏移为0
}
#define SLOPE_GAIN 2.0f
void line_follow_control(void)
{		
	 if (loss_track(mt9v03x_image))  // 图像失踪，立即保护
    {
        set_motor_independent(0, 0);  // 停车
        return;
    }

float deviation = center_weighted_deviation(image_h - 50, image_h -35);

mpu6050_get_gyro();  // 每次更新角速度
float gyro_z_raw = -mpu6050_gyro_transition(mpu6050_gyro_z) + OFFSET;  // 转为 °/s 单位
float gyro_z_dps = get_filtered_gyro_z(gyro_z_raw)* 0.1f; 
// 趋势斜率
//float slope = get_center_slope();

// 组合PD控制：横向偏差 + 趋势修正 + 微分
float delta_speed =
    params.Kp_dir * deviation * (0.3f + 0.7f * fabsf(deviation) / (image_w / 2.0f)) 
  - params.Kd_dir * gyro_z_dps;
int16 target_speed_L = limit_pwm(params.base_speed - delta_speed, MAX_DUTY);
int16 target_speed_R = limit_pwm(params.base_speed + delta_speed, MAX_DUTY);
set_motor_independent(target_speed_L,target_speed_R);

}
void speed_control(void)
{
    
}

void pit_handler(void)
{static uint8 frame_counter = 0;
           encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数

		line_follow_control();
}