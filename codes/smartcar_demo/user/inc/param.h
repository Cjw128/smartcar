// param.h
#ifndef __PARAM_H
#define __PARAM_H

typedef struct {
    float Kp_dir;
    float Kp_slope;
    float Kd_dir;
} ParamStruct;

extern ParamStruct params;
void param_flash_write(void);
void param_flash_read(void);

#endif
