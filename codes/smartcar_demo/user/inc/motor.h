#ifndef _motor_h
#define _motor_h

#include "zf_common_headfile.h"


void set_motor(int duty);
void pit_handler(void);
void encoder_init(void);
void motor_init(void);
void set_motor_independent(int duty_L, int duty_R);
void pit_handler(void);
void line_follow_control(void);
extern volatile uint8 image_ready_flag;
#endif