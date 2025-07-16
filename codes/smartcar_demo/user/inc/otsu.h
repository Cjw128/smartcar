#ifndef _otsu_h
#define _otsu_h
#include "zf_common_headfile.h"

//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
uint8 loss_track(unsigned char in_image[MT9V03X_H][MT9V03X_W]);
extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
extern uint8 l_border[image_h];//左线数组
extern uint8 r_border[image_h];//右线数组



uint8 otsu_find_threshold(uint8 *image, uint16 width, uint16 height);
void highlight_processing(uint8 *image, uint16 width, uint16 height, uint8 threshold);
#endif
