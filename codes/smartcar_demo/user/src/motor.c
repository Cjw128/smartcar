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

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output;
    float last_error;
    float prev_error;
    float last_i;
} PID_INCREMENT_TypeDef;

static PID_INCREMENT_TypeDef pid_speed_L_inc = {0};
static PID_INCREMENT_TypeDef pid_speed_R_inc = {0};
float pid_increment(PID_INCREMENT_TypeDef *pid, float target, float current, float limit, float kp, float ki, float kd)
{
    float error = target - current;
    
    float p_term = kp * (error - pid->last_error);
    float i_term = ki * error * 0.9f + pid->last_i * 0.1f;       
    float d_term = kd * (error - 2 * pid->last_error + pid->prev_error);
    float increment = p_term + i_term + d_term;
    pid->output += increment;
    pid->last_i = i_term;
    // 更新误差历史
    pid->prev_error = pid->last_error;
    pid->last_error = error;

    // 输出限幅
    if (pid->output > limit) pid->output = limit;
    else if (pid->output < -limit) pid->output = -limit;
    
    return pid->output;
}
int16 unified_speed_pid_control(PID_t *pid, int16 target_speed, int16 left_speed, int16 right_speed)
{
    // 计算左右轮实际平均速度
    int16 actual_avg_speed = (left_speed + right_speed) / 2;
    
    // 计算速度误差
    float error = target_speed - actual_avg_speed;
    
    // 积分项更新与限幅
    pid->integral += error;
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    
    // 微分项计算
    float derivative = error - pid->last_error;
    pid->last_error = error;
    
    // PID输出计算与限幅
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    return limit_pwm((int16)output, MAX_DUTY);
}
// 初始化电机引脚及PWM
void motor_init(void)
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, 17000, 0);

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_R, 17000, 0);
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
        pwm_set_duty(PWM_L, duty_L * 6000 / 100);
    }
    else
    {
        gpio_set_level(DIR_L, GPIO_HIGH);
        pwm_set_duty(PWM_L, -duty_L * 6000 / 100);
    }

    if (duty_R >= 0)
    {
        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, duty_R * 6000 / 100);
    }
    else
    {
        gpio_set_level(DIR_R, GPIO_HIGH);
        pwm_set_duty(PWM_R, -duty_R * 6000 / 100);
    }
}
int go_on_flag = 1;
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
		go_on_flag = 0;
}

// PID参数初始化
void pid_speed_init_from_params(void)
{
   

    // 增量式PID初始化
    pid_speed_L_inc.Kp = params.kp_speed;
    pid_speed_L_inc.Ki = params.ki_speed;
    pid_speed_L_inc.Kd = params.kd_speed;
    pid_speed_L_inc.last_error = 0;
    pid_speed_L_inc.prev_error = 0;
    pid_speed_L_inc.output = 0;

    pid_speed_R_inc.Kp = params.kp_speed;
    pid_speed_R_inc.Ki = params.ki_speed;
    pid_speed_R_inc.Kd = params.kd_speed;
    pid_speed_R_inc.last_error = 0;
    pid_speed_R_inc.prev_error = 0;
    pid_speed_R_inc.output = 0;
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
float last_deviation = 0;

// 主巡线控制函数，计算左右速度目标，调用电机控制
void line_follow_control(void)
{
    if (loss_track(mt9v03x_image))  // 失线保护
    {
        set_motor_independent(0, 0);
				go_on_flag = 0;
        return;
    }

    if (detect_zebra_cross(mt9v03x_image))
{
    set_motor_independent(0, 0);  // 停车
    return;  // 退出巡线控制
}

    measured_speed_L = encoder_data_1;
    measured_speed_R = encoder_data_2;
 mpu6050_get_gyro();
    float gyro_z_raw = -mpu6050_gyro_transition(mpu6050_gyro_z) + OFFSET;
    float gyro_z_dps = get_filtered_gyro_z(gyro_z_raw) * 0.5f;
    float deviation = center_weighted_deviation(image_h - 90, image_h - 75);
	float abs_dev = fabsf(deviation);
    float diff = deviation - last_deviation;
    last_deviation = deviation;

    float steer_feedforward = params.kf_dir * deviation;

    float delta_speed = 
          params.Kp_dir * deviation * (0.3f + 0.7f * fabsf(deviation) / (image_w / 2.0f))
        - params.Kd_dir * gyro_z_dps//*fabs(gyro_z_dps);
        - params.kd_diff *diff
        + steer_feedforward;

        if (abs_dev <5.0f)
{adaptive_base_speed = params.base_speed * 1.0f;	

}
 else{adaptive_base_speed = params.base_speed;
 }
    left_target_speed = adaptive_base_speed + delta_speed;
    right_target_speed = adaptive_base_speed - delta_speed;

    float ff_L = params.kf_speed * left_target_speed;
    float ff_R = params.kf_speed * right_target_speed;

    // 使用增量式PID控制
    int16 duty_L = (int16)pid_increment(&pid_speed_L_inc, left_target_speed, measured_speed_L, MAX_DUTY, params.kp_speed, params.ki_speed, params.kd_speed) + ff_L;
    int16 duty_R = (int16)pid_increment(&pid_speed_R_inc, right_target_speed, measured_speed_R, MAX_DUTY, params.kp_speed, params.ki_speed, params.kd_speed) + ff_R;
    set_motor_independent(duty_L, duty_R);
	//printf("%d,%d\n",measured_speed_L,measured_speed_R);
}
volatile uint8 flag_do_control = 0;



void steering_loop(void)            //转向环控制
{
    // 采集传感器、计算偏差
    float deviation = center_weighted_deviation(image_h - 90, image_h - 75);
    float diff = deviation - last_deviation;
    last_deviation = deviation;
    float gyro_z_raw = -mpu6050_gyro_transition(mpu6050_gyro_z) + OFFSET;
    float gyro_z_dps = get_filtered_gyro_z(gyro_z_raw) * 0.5f;

    float steer_feedforward = params.kf_dir * deviation;
    float delta_speed = 
        params.Kp_dir * deviation * (0.3f + 0.7f * fabsf(deviation) / (image_w / 2.0f))
        - params.Kd_dir * gyro_z_dps
        - params.kd_diff * diff
        + steer_feedforward;

    float abs_dev = fabsf(deviation);
    if (abs_dev < 5.0f)
        adaptive_base_speed = params.base_speed * 1.0f;
    else
        adaptive_base_speed = params.base_speed;

    left_target_speed = adaptive_base_speed + delta_speed;
    right_target_speed = adaptive_base_speed - delta_speed;
}

void speed_loop(void)               //速度环控制
{
    if (loss_track(mt9v03x_image))  // 失线保护
    {
        set_motor_independent(0, 0);
				go_on_flag = 0;
        return;
    }
    
    measured_speed_L = encoder_data_1;
    measured_speed_R = encoder_data_2;

    float ff_L = params.kf_speed * left_target_speed;
    float ff_R = params.kf_speed * right_target_speed;

    int16 duty_L = (int16)pid_increment(&pid_speed_L_inc, left_target_speed, measured_speed_L, MAX_DUTY, params.kp_speed, params.ki_speed, params.kd_speed) + ff_L;
    int16 duty_R = (int16)pid_increment(&pid_speed_R_inc, right_target_speed, measured_speed_R, MAX_DUTY, params.kp_speed, params.ki_speed, params.kd_speed) + ff_R;
    set_motor_independent(duty_L, duty_R);
}



void pit_handler(void)              //速度环中断函数
{
   
// line_follow_control();
    speed_loop();
}
void encoder_sample_handler(void)   //转向环中断函数
{
    //encoder_data_1 = encoder_get_count(ENCODER_1);
    //encoder_data_2 = -encoder_get_count(ENCODER_2);
    //encoder_clear_count(ENCODER_1);
    //encoder_clear_count(ENCODER_2);
    steering_loop();

}