#include "zf_common_headfile.h"
#include "OTSU.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8 image_threshold = 128;
uint8 original_image[MT9V03X_H][MT9V03X_W];

int abs(int value)
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

void Get_image(uint8(*mt9v03x_image)[MT9V03X_W])
{
#define use_num		1	//1就是不压缩，2就是压缩一倍	
	uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < MT9V03X_H; i += use_num)          //
    {
        for (j = 0; j <MT9V03X_W; j += use_num)     //
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
// 函数简介     对原始图像过曝区域处理
// 参数说明     *image          总钻风图像数组指针，可以是复制用的图像
// 参数说明     width           图像宽度，默认为240
// 参数说明     height          图像高度，默认为180
// 返回参数     threshold       自定阈值
// 使用示例     highlight_processing((uint8*)image1,MT9V03X_W,MT9V03X_H,250);
// 备注信息     image1建议使用memcpy先复制一遍原始图像
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     储存二值化图像
// 参数说明     *image          总钻风图像数组指针
// 参数说明     width           图像宽度
// 参数说明     height          图像高度
// 参数说明     threshold       二值化阈值，大津法得到或固定阈值
// 使用示例     save_processed_image((uint8*)image1,MT9V03X_W,MT9V03X_H,250);
//-------------------------------------------------------------------------------------------------------------------
void save_processed_image(uint8 *image, uint16 width, uint16 height, uint8 threshold)
{
    for (int i = 0; i < height; i++) {         // 遍历行
        for (int j = 0; j < width; j++) {      // 遍历列
            uint8 pixel = image[i * width + j]; 
            if (pixel < threshold) {
                image[i * width + j] = 0;
            } else {
                image[i * width + j] = 255; 
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找初始边界点（黑白跳变点）
// 参数说明     *image          总钻风图像数组指针
// 参数说明     width           图像宽度
// 参数说明     height          图像高度
// 使用示例     
//-------------------------------------------------------------------------------------------------------------------
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 image[MT9V03X_H][MT9V03X_W], uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = MT9V03X_W / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (original_image[start_row][i] == 255 && original_image[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = MT9V03X_W / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (original_image[start_row][i] == 255 && original_image[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给图像画黑框
// 参数说明     *image          总钻风图像数组指针
// 使用示例     image_draw_rectan(uint8(*image)[MT9V03X_W])
//-------------------------------------------------------------------------------------------------------------------
void image_draw_rectan(uint8(*image)[MT9V03X_W])
{

	uint8 i = 0;
	for (i = 0; i < MT9V03X_H; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][MT9V03X_W - 1] = 0;
		image[i][MT9V03X_W - 2] = 0;

	}
	for (i = 0; i < MT9V03X_W; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[image_h-1][i] = 0;

	}
}


//-------------------------------------------------------------------------------------------------------------------
//函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
//							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)
//
// 功能说明    八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
// 参数说明    break_flag_r			最多需要循环的次数
// 参数说明   (*image)[image_w]		需要进行找点的图像数组，必须是二值图,填入数组名称即可
//特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
// 参数说明   *l_stastic			统计左边数据，用来输入初始数组成员的序号和取出循环次数
// 参数说明   *r_stastic			统计右边数据，用来输入初始数组成员的序号和取出循环次数
// 参数说明    l_start_x			左边起点横坐标
// 参数说明    l_start_y			左边起点纵坐标
// 参数说明    r_start_x			右边起点横坐标
// 参数说明    r_start_y			右边起点纵坐标
// 参数说明    hightest				循环结束所得到的最高高度
// 备    注：
// example：
//	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
//				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
//-------------------------------------------------------------------------------------------------------------------

#define USE_num	300	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[MT9V03X_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		//左边
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
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			//printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			printf("\n如果左边比右边高了，左边等待右边\n");	
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
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


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从八邻域边界提取需要的边线
// 参数说明     data_stastics_l          左边找到点的个数，已定义
// 使用示例     get_left(uint16 data_stastics_l)
//-------------------------------------------------------------------------------------------------------------------

uint8 l_border[MT9V03X_H];//左线数组
uint8 r_border[MT9V03X_H];//右线数组
uint8 center_line[MT9V03X_H];//中线数组
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	//初始化
	for (i = 0;i<MT9V03X_H;i++)
	{
		l_border[i] = border_min;
	}
	h = MT9V03X_H - 2;
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

void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < MT9V03X_H; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = MT9V03X_H - 2;
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
void image_process(uint8(*image)[MT9V03X_W])
{
    uint16 i;
    uint8 hightest = 10;

    data_stastics_l = 0;
    data_stastics_r = 0;

    // 拷贝原图
    memcpy(original_image, mt9v03x_image, MT9V03X_W * MT9V03X_H);

    // 高亮处理（防过曝）
    highlight_processing((uint8*)original_image, MT9V03X_W, MT9V03X_H, 245);

    // 大津法找阈值 + 二值化
    uint8 threshold = otsu_find_threshold((uint8*)original_image, MT9V03X_W, MT9V03X_H);
    save_processed_image((uint8*)original_image, MT9V03X_W, MT9V03X_H, threshold);

    // 给图像加黑框
    image_draw_rectan(original_image);  // 必须对二值图加黑框

    // 起点查找 + 八邻域爬线
    if (get_start_point(original_image,MT9V03X_H - 2)) {
        printf("正在开始八邻域\n");
        search_l_r(USE_num, original_image, &data_stastics_l, &data_stastics_r,
                   start_point_l[0], start_point_l[1],
                   start_point_r[0], start_point_r[1], &hightest);
        printf("八邻域已结束\n");

        get_left(data_stastics_l);
        get_right(data_stastics_r);
    }
}