#ifndef _menu_h
#define _menu_h

#include "zf_common_headfile.h"

int menu1(void);
int menu2_1(void);
int menu2_2(void);

int menu_param_config(void);
typedef struct
{
    float Kp_dir;
    float kp_speed;
    float Kd_dir;
    float base_speed;
    float target_speed;
    float ki_speed;
    float kd_speed;
    float kd_diff;
    float kf_speed;  
    float kf_dir;    
} param_config_t;

extern param_config_t params;

void param_flash_write(void);
void param_flash_read(void);
int menu_param_config(void);
int menu_param_feedforward(void);
#endif