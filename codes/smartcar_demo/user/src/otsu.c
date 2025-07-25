
#include "zf_common_headfile.h"
#include "motor.h"
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W]; 
#define PROTECT_ROW_COUNT 10        // 检测底部的行数
#define PROTECT_WHITE_PIXEL_MIN 50 // 最低白色像素阈值
//------------------------------------------------------------------------------------------------------------------
// 函数名称			int my_abs(int value)
// 功能说明			求绝对值
// 参数说明
// 返回参数			绝对值
//------------------------------------------------------------------------------------------------------------------
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

//------------------------------------------------------------------------------------------------------------------
// 函数名称			int16 limit(int16 x, int16 y)
// 功能说明			求x,y中的最小值
// 参数说明
// 返回参数			返回两值中的最小值
//------------------------------------------------------------------------------------------------------------------
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}



uint8 original_image[image_h][image_w];
uint8 image_thereshold;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
// 函数简介      获得一副灰度图像
// 参数说明      uint8(*mt9v03x_image)[image_w] 	二维数组指针
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num		1	//1就是不压缩，2就是压缩一倍	
	uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
			line++;
        }
        line = 0;
        row++;
    }
}
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
//-------------------------------------------------------------------------------------------------------------------
// 	函数简介     对原始图像过曝区域处理
// 	参数说明     *image          总钻风图像数组指针，可以是复制用的图像
// 	参数说明     width           图像宽度，默认为240
// 	参数说明     height          图像高度，默认为180
// 	返回参数     threshold       自定阈值
// 	使用示例     highlight_processing((uint8*)image1,MT9V03X_W,MT9V03X_H,250);
// 	备注信息     建议先复制一遍原始图像
//-------------------------------------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------------------------------------
//  函数简介      图像二值化 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
{
  
	uint8 i,j;
 image_thereshold = otsu_find_threshold(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


//------------------------------------------------------------------------------------------------------------------
//	函数名称：void get_start_point(uint8 start_row)
//	功能说明：寻找两个边界的边界点作为八邻域循环的起始点
//	参数说明：输入任意行数
//------------------------------------------------------------------------------------------------------------------
uint8 start_point_l[2] = { 0 };
uint8 start_point_r[2] = { 0 };
uint8 get_start_point(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
		{
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		return 0;
	} 
}

//------------------------------------------------------------------------------------------------------------------
//函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
//							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

//	功能说明										八邻域爬线
//	参数说明
//	参数说明	break_flag_r				最多需要循环的次数
//	参数说明	(*image)[image_w]		需要进行找点的图像数组，必须是二值图,填入数组名称即可
//	参数说明	*l_stastic					统计左边数据，用来输入初始数组成员的序号和取出循环次数
//	参数说明	*r_stastic					统计右边数据，用来输入初始数组成员的序号和取出循环次数
//	参数说明	l_start_x						左边起点横坐标
//	参数说明	l_start_y						左边起点纵坐标
//	参数说明	r_start_x						右边起点横坐标
//	参数说明	r_start_y						右边起点纵坐标
//	参数说明	hightest						循环结束所得到的最高高度
//	example：
//	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
//				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
//------------------------------------------------------------------------------------------------------------------
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;
uint16 data_stastics_r = 0;
uint8 hightest = 0; 
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };
	uint8 index_r = 0;
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };

	l_data_statics = *l_stastic;
	r_data_statics = *r_stastic;

	center_point_l[0] = l_start_x;
	center_point_l[1] = l_start_y;
	center_point_r[0] = r_start_x;
	center_point_r[1] = r_start_y;

	while (break_flag--)
	{

		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}

			if (index_l)
			{
				//更新坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1])
					{
						center_point_l[0] = temp_l[j][0];//x
						center_point_l[1] = temp_l[j][1];//y
					}
				}
			}

		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			continue;
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))
		{
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;

		index_r = 0;
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;
			temp_r[i][1] = 0;
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;
				dir_r[r_data_statics - 1] = (i);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[image_h];//左线数组
uint8 r_border[image_h];//右线数组
uint8 center_line[image_h];//中线数组
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	//初始化
	for (i = 0;i<image_h;i++)
	{
		l_border[i] = border_min;
	}
	h = image_h - 2;
	//左边
	for (j = 0; j < total_L; j++)
	{
		//printf("%d\n", j);
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h - 2;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//黑

			}

		}
	}

}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

	uint8 i = 0;
	for (i = 0; i < image_h; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[image_h-1][i] = 0;

	}
}// 失线检测
uint8 loss_track(unsigned char in_image[MT9V03X_H][MT9V03X_W])
{
    int sum = 0;
    int count = 0;
    
    for (int j = 0; j < 5; j++)
    {
        for (int i = 0; i < 30; i++)
        {
            int col = MT9V03X_W / 2 - 15 + i;  
            int row = MT9V03X_H - 1 - j;       
            if (col >= 0 && col < MT9V03X_W && row >= 0 && row < MT9V03X_H)
            {
                sum += in_image[row][col];
                count++;
            }
        }
    }
    
    if (count == 0) return 0;
    int average = sum / count;
    
    if (average < 120)  
    {
        return 1;  // 出界
    }
    else
    {
        return 0;  // 正常
    }
}
void avoid_obstacle(void)
{
    int black_count = 0;
    for (int i = image_h - 10; i < image_w; i++) {
        for (int j = 0; j <image_w; j++) {
            if (bin_image[i][j] == 0) black_count++;
        }
    }

    static int last_black_count = 0;
    int delta = black_count - last_black_count;
    last_black_count = black_count;

    if (delta > 500)  // 黑色像素突增阈值，可调
    {
        if (image_w / 2 - center_line[image_h - 5] > 10) {
            // 中线偏左，说明障碍在右，向左避障
            for (int i = image_h - 30; i < image_h; i++) {
                r_border[i] = (r_border[i] > 20) ? r_border[i] - 20 : 0;
            }
        } else if (center_line[image_h - 5] - image_w / 2 > 10) {
            // 中线偏右，障碍在左，向右避障
            for (int i = image_h - 30; i < image_h; i++) {
                l_border[i] = (l_border[i] < image_w - 20) ? l_border[i] + 20 : image_w - 1;
            }
        }
    }
}

/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
uint16 i;
uint8 hightest = 0;
Get_image(mt9v03x_image);
highlight_processing(*original_image,image_w,image_h,250);
turn_to_bin();
/*提取赛道边界*/
image_filter(bin_image);//滤波
image_draw_rectan(bin_image);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
	search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
	//处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}

// ========== 环岛状态机 ==========
//    static uint8 meetRingFlag = 0, enterRingFlag = 0, leaveRingFlag = 0, ringSide = 0;
//    uint8 jumpFlagL = 0, jumpFlagR = 0, pointType = 0, pointSide = 0;
//    uint16 pointX = 0, pointY = 0;

//    for (i = image_h - 2; i > image_h / 3; i--)
//    {
//        if (l_border[i + 1] - l_border[i] > 20) // 左A跳变点
//        {
//            jumpFlagL = 1;
//            pointX = l_border[i + 1];
//            pointY = i + 1;
//            pointSide = 1;
//            pointType = 1;
//        }
//        else if (l_border[i] - l_border[i + 1] > 20) // 左V跳变点
//        {
//            jumpFlagL = 1;
//            pointX = l_border[i];
//            pointY = i;
//            pointSide = 1;
//            pointType = 2;
//        }

//        if (r_border[i] - r_border[i + 1] > 20) // 右A跳变点
//        {
//            jumpFlagR = 1;
//            pointX = r_border[i + 1];
//            pointY = i + 1;
//            pointSide = 2;
//            pointType = 1;
//        }
//        else if (r_border[i + 1] - r_border[i] > 20) // 右V跳变点
//        {
//            jumpFlagR = 1;
//            pointX = r_border[i];
//            pointY = i;
//            pointSide = 2;
//            pointType = 2;
//        }

//        if (jumpFlagL ^ jumpFlagR) break;
//    }

//    // 补线点
//    uint16 topX = image_w / 2, topY = 0;
//    uint16 l_bot_x = 0, l_bot_y = image_h - 1;
//    uint16 r_bot_x = image_w - 1, r_bot_y = image_h - 1;
//    float stepLength = 0;

//    if (jumpFlagL ^ jumpFlagR)
//    {
//        if (pointType == 1) // A字跳变点
//        {
//            if (enterRingFlag)
//            {
//                leaveRingFlag = 1;
//                if (ringSide == 1) // 左环补右边
//                {
//                    stepLength = (float)(topX - pointX) / (float)(topY - pointY);
//                    for (i = 0; i < topY - pointY; i++)
//                    {
//                        r_border[topY - i] = topX - (int)(i * stepLength);
//                        bin_image[topY - i][r_border[topY - i]] =
//                        bin_image[topY - i][r_border[topY - i] - 1] =
//                        bin_image[topY - i][r_border[topY - i] + 1] = 0;
//                    }
//                }
//                else if (ringSide == 2) // 右环补左边
//                {
//                    stepLength = (float)(pointX - topX) / (float)(topY - pointY);
//                    for (i = 0; i < topY - pointY; i++)
//                    {
//                        l_border[topY - i] = topX + (int)(i * stepLength);
//                        bin_image[topY - i][l_border[topY - i]] =
//                        bin_image[topY - i][l_border[topY - i] - 1] =
//                        bin_image[topY - i][l_border[topY - i] + 1] = 0;
//                    }
//                }
//            }
//            else if (!meetRingFlag)
//            {
//                meetRingFlag = 1;
//                ringSide = pointSide;
//            }
//        }
//        else if (pointType == 2) // V字跳变点
//        {
//            if (leaveRingFlag) // 过环
//            {
//                meetRingFlag = 0;
//                enterRingFlag = 0;
//                leaveRingFlag = 0;
//                if (ringSide == 1)
//                {
//                    stepLength = (float)(pointX - l_bot_x) / (float)(l_bot_y - pointY);
//                    for (i = 0; i < l_bot_y - pointY; i++)
//                    {
//                        l_border[l_bot_y - i] = l_bot_x + (int)(i * stepLength);
//                        bin_image[l_bot_y - i][l_border[l_bot_y - i]] =
//                        bin_image[l_bot_y - i][l_border[l_bot_y - i] - 1] =
//                        bin_image[l_bot_y - i][l_border[l_bot_y - i] + 1] = 0;
//                    }
//                }
//                else if (ringSide == 2)
//                {
//                    stepLength = (float)(r_bot_x - pointX) / (float)(r_bot_y - pointY);
//                    for (i = 0; i < r_bot_y - pointY; i++)
//                    {
//                        r_border[r_bot_y - i] = r_bot_x - (int)(i * stepLength);
//                        bin_image[r_bot_y - i][r_border[r_bot_y - i]] =
//                        bin_image[r_bot_y - i][r_border[r_bot_y - i] - 1] =
//                        bin_image[r_bot_y - i][r_border[r_bot_y - i] + 1] = 0;
//                    }
//                }
//            }
//            else if (meetRingFlag) // 入环
//{
//    enterRingFlag = 1;

//    if (ringSide == 1) // 左环：补右边线（右下 → 入环跳变点）
//    {
//        stepLength = (float)(r_bot_x - pointX) / (float)(r_bot_y - pointY);
//        for (i = 0; i < r_bot_y - pointY; i++)
//        {
//            r_border[r_bot_y - i] = r_bot_x - (int)(i * stepLength);
//            bin_image[r_bot_y - i][r_border[r_bot_y - i]] =
//            bin_image[r_bot_y - i][r_border[r_bot_y - i] - 1] =
//            bin_image[r_bot_y - i][r_border[r_bot_y - i] + 1] = 0;
//        }
//    }
//    else if (ringSide == 2) // ✅ 右环：应补左边线（左下 → 入环跳变点）
//    {
//        stepLength = (float)(pointX - l_bot_x) / (float)(l_bot_y - pointY);
//        for (i = 0; i < l_bot_y - pointY; i++)
//        {
//            l_border[l_bot_y - i] = l_bot_x + (int)(i * stepLength);
//            bin_image[l_bot_y - i][l_border[l_bot_y - i]] =
//            bin_image[l_bot_y - i][l_border[l_bot_y - i] - 1] =
//            bin_image[l_bot_y - i][l_border[l_bot_y - i] + 1] = 0;
//        }
//    }
//}
//       }
//    }
avoid_obstacle();
  

extern void ips200_displayimage032_zoom(uint8 *img, uint16 src_w, uint16 src_h,
                                        uint16 dst_w, uint16 dst_h, uint16 x, uint16 y);
// 计算中线
  // 重新计算中线
//ips200_clear();
    for (i = hightest; i < image_h; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1;
        bin_image[i][center_line[i]] =
        bin_image[i][center_line[i] - 1] =
        bin_image[i][center_line[i] + 1] = 0;
    }

//    for (i = 0; i < data_stastics_l; i++)
//        ips200_draw_point(points_l[i][0] + 2, points_l[i][1], RGB565_BLUE);

//    for (i = 0; i < data_stastics_r; i++)
//        ips200_draw_point(points_r[i][0] - 2, points_r[i][1], RGB565_RED);

//    for (i = hightest; i < image_h - 1; i++)
//    {
//        ips200_draw_point(center_line[i], i, RGB565_BLUE);
//        ips200_draw_point(l_border[i], i, RGB565_GREEN);
//        ips200_draw_point(r_border[i], i, RGB565_GREEN);
//    }

    // 可选整图显示
    // ips200_show_gray_image(0, 0, (const uint8 *)bin_image, MT9V03X_W, MT9V03X_H, 188, 120, 0);



	}


