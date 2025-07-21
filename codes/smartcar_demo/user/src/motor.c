#include "zf_common_headfile.h"
#include "motor.h"
#include "otsu.h"
#include <math.h>
#include "menu.h"

/***************************************** 编码器部分 **********************************************/
#define ENCODER_1                   (TIM3_ENCODER)
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)

#define ENCODER_2                   (TIM4_ENCODER)
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

#define PIT                         (TIM6_PIT)  // 周期中断

/***************************************** 电机部分 **********************************************/
#define MAX_DUTY            (90)   // 最大占空比
#define DIR_L               (A0)
#define PWM_L               (TIM5_PWM_CH2_A1)

#define DIR_R               (A2)
#define PWM_R               (TIM5_PWM_CH4_A3)

/***************************************** 外部参数 **********************************************/
extern param_config_t params;             // 参数结构体
extern uint8 center_line[image_h];        // 中线数组

/***************************************** 变量定义 **********************************************/
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;

// PID结构体，用于速度环
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
} PID_t;

static PID_t pid_speed_L;
static PID_t pid_speed_R;
int16 left_target_speed = 0;
int16 right_target_speed = 0;
int16 measured_speed_L = 0;
int16 measured_speed_R = 0;

/***************************************** 函数声明 **********************************************/
int16 limit_pwm(int16 x, int limit);
int16 speed_pid_control(PID_t *pid, int16 target_speed, int16 measured_speed);
float get_filtered_gyro_z(float raw_gyro_z_dps);

/***************************************** 函数实现 **********************************************/
// 限幅函数
int16 limit_pwm(int16 x, int limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

// PID速度控制计算
int16 speed_pid_control(PID_t *pid, int16 target_speed, int16 measured_speed)
{
    float error = target_speed - measured_speed;
    pid->integral += error;

    // 积分限幅防止积分饱和
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;

    float derivative = error - pid->last_error;
    pid->last_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    return limit_pwm((int16)output, MAX_DUTY);
}

// 初始化电机引脚及PWM
void motor_init(void)
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, 10000, 0);

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_R, 10000, 0);
}

// 初始化编码器
void encoder_init(void)
{
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
}

// 设置电机占空比和方向
void set_motor_independent(int duty_L, int duty_R)
{
    duty_L = limit_pwm(duty_L, MAX_DUTY);
    duty_R = limit_pwm(duty_R, MAX_DUTY);

    if (duty_L >= 0)
    {
        gpio_set_level(DIR_L, GPIO_LOW);
        pwm_set_duty(PWM_L, duty_L * 4900 / 100);
    }
    else
    {
        gpio_set_level(DIR_L, GPIO_HIGH);
        pwm_set_duty(PWM_L, -duty_L * 4900 / 100);
    }

    if (duty_R >= 0)
    {
        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, duty_R * 4900 / 100);
    }
    else
    {
        gpio_set_level(DIR_R, GPIO_HIGH);
        pwm_set_duty(PWM_R, -duty_R * 4900 / 100);
    }
}
bool detect_zebra_cross(uint8 (*image)[image_w])
{
    int row_start = image_h - 20;
    int row_end = image_h - 1;
    int column = image_w / 2;  // 检查中线位置（你也可以检查多个列平均）
    int transitions = 0;
    int last_pixel = 0;

    // 初始化为第一行像素值
    last_pixel = image[row_start][column];

    for (int i = row_start + 1; i <= row_end; i++)
    {
        int pixel = image[i][column];

        // 检查黑白交替（像素值从0到255，设一个中间阈值判断）
        if ((last_pixel > 128 && pixel < 128) || (last_pixel < 128 && pixel > 128))
        {
            transitions++;
        }

        last_pixel = pixel;
    }

    // 如果黑白交替超过一定次数，判断为斑马线
    return transitions >= 4;
}

// PID参数初始化
void pid_speed_init_from_params(void)
{
    pid_speed_L.Kp = params.kp_speed;
    pid_speed_L.Ki = params.ki_speed;
    pid_speed_L.Kd = params.kd_speed;
    pid_speed_L.integral = 0;
    pid_speed_L.last_error = 0;

    pid_speed_R.Kp = params.kp_speed;
    pid_speed_R.Ki = params.ki_speed;
    pid_speed_R.Kd = params.kd_speed;
    pid_speed_R.integral = 0;
    pid_speed_R.last_error = 0;
}

// 速度反馈更新
void update_speed_feedback(void)
{
    measured_speed_L = encoder_get_count(ENCODER_1);
    measured_speed_R = encoder_get_count(ENCODER_2);

    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
}

#define OFFSET -6.524390f
static float gyro_z_filtered = 0.0f;
#define GYRO_FILTER_ALPHA  0.15f

float get_filtered_gyro_z(float raw_gyro_z_dps)
{
    gyro_z_filtered = (1.0f - GYRO_FILTER_ALPHA) * gyro_z_filtered + GYRO_FILTER_ALPHA * raw_gyro_z_dps;
    return gyro_z_filtered;
}

// 计算加权中线偏差
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
        if (c > 5 && c < image_w - 5)
        {
            float weight = i - start + 1;
            float offset = (image_w / 2.0f) - c;
            sum += offset * weight;
            weight_sum += weight;
        }
    }

    if (weight_sum > 0)
        return sum / weight_sum;
    else
        return 0;
}
float adaptive_base_speed = 0;  // 当前动态基础速度
int go_on_flag = 1;
// 主巡线控制函数，计算左右速度目标，调用电机控制
void line_follow_control(void)


{
    if (loss_track(mt9v03x_image))  // 失线保护
    {
        set_motor_independent(0, 0);
				go_on_flag = 0;
        return;
    }

//    if (detect_zebra_cross(mt9v03x_image))
//{
//    set_motor_independent(0, 0);  // 停车
//    return;  // 退出巡线控制
//}
    float deviation = center_weighted_deviation(image_h - 100, image_h - 70);
	float abs_dev = fabsf(deviation);
if (abs_dev < 10.0f)
    adaptive_base_speed = params.base_speed * 1.0f;
else if (abs_dev < 25.0f)
    adaptive_base_speed = params.base_speed * 0.8f;
else
    adaptive_base_speed = params.base_speed * 0.6f;
    mpu6050_get_gyro();
    float gyro_z_raw = -mpu6050_gyro_transition(mpu6050_gyro_z) + OFFSET;
    float gyro_z_dps = get_filtered_gyro_z(gyro_z_raw) * 0.1f;

    float delta_speed = 
          params.Kp_dir * deviation * (0.3f + 0.7f * fabsf(deviation) / (image_w / 2.0f))
        - params.Kd_dir * gyro_z_dps;//*fabs(gyro_z_dps);

    left_target_speed = limit_pwm(adaptive_base_speed - delta_speed, MAX_DUTY);
    right_target_speed = limit_pwm(adaptive_base_speed + delta_speed, MAX_DUTY);

    // PID速度环计算PWM
    int16 pwm_L = speed_pid_control(&pid_speed_L, left_target_speed, measured_speed_L);
    int16 pwm_R = speed_pid_control(&pid_speed_R, right_target_speed, measured_speed_R);
    set_motor_independent(left_target_speed, right_target_speed);

}
volatile uint8 flag_do_control = 0;
// PIT定时器中断处理函数，定期更新速度反馈并执行控制
void pit_handler(void)
{
//     encoder_data_1 = encoder_get_count(ENCODER_1);
//    encoder_data_2 = -encoder_get_count(ENCODER_2);
//    encoder_clear_count(ENCODER_1);
//    encoder_clear_count(ENCODER_2);

    measured_speed_L = encoder_data_1;
    measured_speed_R = encoder_data_2;

 line_follow_control();

}