#ifndef _otsu_h
#define _otsu_h

#include "zf_common_headfile.h"
uint8 otsu_find_threshold(uint8 *image, uint16 width, uint16 height);
void highlight_processing(uint8 *image, uint16 width, uint16 height, uint8 threshold);
#endif
