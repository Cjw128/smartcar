#ifndef _motor_h
#define _motor_h

#include "zf_common_headfile.h"

extern int16 left_target_speed;
extern int16 right_target_speed;
extern int16 measured_speed_L;
extern int16 measured_speed_R;

void set_motor(int duty);
void pit_handler(void);
void encoder_init(void);
void motor_init(void);
void set_motor_independent(int duty_L, int duty_R);
void pit_handler(void);
void line_follow_control(void);
extern volatile uint8 image_ready_flag;
void calibrate_gyro_offset(void);
void encoder_sample_handler(void);
#endif