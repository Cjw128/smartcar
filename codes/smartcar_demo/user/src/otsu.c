#include "zf_common_headfile.h"
#include "OTSU.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8 image_threshold = 128;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法寻找动态阈值
// 参数说明     *image          总钻风图像数组指针
// 参数说明     width           图像宽度，默认为240
// 参数说明     height          图像高度，默认为180
// 返回参数     threshold       返回大津法找到的阈值，用于后续巡线
// 使用示例     uint8 threshold = otsu_find_threshold((uint8*)mt9v03x_image,MT9V03X_W,MT9V03X_H);               // 大津法寻找阈值
// 备注信息     数组的数据由外部传入
//-------------------------------------------------------------------------------------------------------------------
uint8 otsu_find_threshold(uint8 *image, uint16 width, uint16 height) {
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};
    float pixelPro[GrayScale] = {0};
    int i, j;
    int sumpix = width * height;
    uint8 threshold = 0;
    uint8* data = image;

    // 统计灰度值
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            pixelCount[(int)data[i * width + j]]++;
        }
    }

    // 计算灰度比例和总平均灰度
    float u = 0.0;  // 总平均灰度
    for (i = 0; i < GrayScale; i++) {
        pixelPro[i] = (float)pixelCount[i] / sumpix;
        u += i * pixelPro[i];
    }

    // 寻找最优阈值
    float maxVariance = 0.0;
    float w0 = 0.0;        // 前景像素比例
    float avgValue = 0.0;   // 前景灰度值总和
    
    for (i = 0; i < GrayScale; i++) {
        w0 += pixelPro[i];             // 更新前景比例
        avgValue += i * pixelPro[i];   // 更新前景灰度总和

        if (w0 < 1e-5 || w0 > 1 - 1e-5) {
            continue; // 防止除以0
        }

        //类间方差计算公式 ↓
        float numerator = avgValue - u * w0;
        float variance = numerator * numerator / (w0 * (1.0f - w0));

        if (variance > maxVariance) {
            maxVariance = variance;
            threshold = i;
        }
    }
    return threshold;
}

// 过曝区域处理
void highlight_processing(uint8 *image, uint16 width, uint16 height, uint8 threshold) {
    for (int y = 1; y < height-1; y++) {
        for (int x = 1; x < width-1; x++) {
            if (image[y*width + x] > threshold) {  // 如果像素过亮
                // 计算3x3邻域的平均值
                int sum = 0;
                int count = 0;
                for (int ky = -1; ky <= 1; ky++) {
                    for (int kx = -1; kx <= 1; kx++) {
                        if (image[(y+ky)*width + (x+kx)] <= threshold) {
                            sum += image[(y+ky)*width + (x+kx)];
                            count++;
                        }
                    }
                }
                if (count > 0) {
                    image[y*width + x] = sum / count;  // 设置为邻域平均值
                }
            }
        }
    }
}