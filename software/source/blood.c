
/*******************************************************************************
 * @file 	 		blood.c
 * @author  	日常里的奇迹	@bilibili
 * @date    	2022-11-26
 * @brief			血氧心率算法
********************************************************************************/

#include "blood.h"
#include "MAX30102.h"
#include "math.h"
#include "algorithm.h"
#include "GUI.h"

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

//存放血氧数据
static float spO2_num;
//存放心率数据
static float Heart_Rate;

//存放参考代码心率数据
static float Heart_Rate1;

//用于 FFT
struct compx s1[BUFFER_SIZE+16];           	
struct compx s2[BUFFER_SIZE+16];           	


//用于 临时存储数据
static	float temp_buffer[BUFFER_SIZE];

/*******************************************************************************
 * Local function 
 ******************************************************************************/
 
 
 
/**********************************************************************
 * @brief 	 		从中位数开始找到大于阈值的数
 * @parameter  	buffer 缓冲区指针
								num 数据个数
								threshold 阈值
 * @return    	uint16_t 从中位数开始找到大于阈值的数或最大值
  ********************************************************************/
static float Find_Melia(uint16_t * buffer,uint16_t num,uint16_t threshold)
{
	uint16_t temp[num];
	for(uint16_t i = 0;i < num;i++)
	{
		temp[i] = buffer[i];
	}
	
	//冒泡排序
	int j,i;
	float tem;
	for (i = 0; i < num-1;i ++)//size-1是因为不用与自己比较，所以比的数就少一个
	{
		int count = 0;
		for (j = 0; j < num-1 - i; j++)	//size-1-i是因为每一趟就会少一个数比较
		{
			if (temp[j] > temp[j+1])//这是升序排法，前一个数和后一个数比较，如果前数大则与后一个数换位置
			{
				tem = temp[j];
				temp[j] = temp[j+1];
				temp[j+1] = tem;
				count = 1;
				
			}
		}
		if (count == 0)			//如果某一趟没有交换位置，则说明已经排好序，直接退出循环
				break;	
	}	
	
	tem = 0;
	uint16_t number = 0;
	//从中位数开始找到大于阈值的数
	for(i = num/2;i < num - 1;i++)
	{			
		if(temp[i] > threshold)
		{
			tem = tem + temp[i];
			number++;
		}
	}
	if(tem == 0)
		tem = temp[num - 1];
	else
		tem = tem / number;
	
	return tem;
}

/**********************************************************************
 * @brief 	 		找出最大值
 * @parameter  	buffer 缓冲区指针
								num 数据个数
 * @return    	uint16_t 最大值
  ********************************************************************/
//static uint16_t Find_Max(uint16_t * buffer,uint16_t num)
//{
//	uint16_t max = buffer[0];
//	for(uint16_t i = 1;i < num;i++)
//	{
//		if(max < buffer[i])
//			max = buffer[i];
//	}
//	return max;
//}

/**********************************************************************
 * @brief 	 		通过过零点计算心率，计算之前需要把把缓冲区信号处理为交流信号
 * @parameter  	
 * @return    	
  ********************************************************************/
static float Heart_Rate_Process(void)
{
	//找出所有(由负到正)过零点的索引 从缓冲区第3个元素开始找，到BUFFER_SIZE - 3为止，
	//因为头两个点和最后两个点没有进行中位数滤波，所以排除以避免干扰
	uint16_t zero_cross_index[BUFFER_SIZE/2 + 1];
	uint16_t zero_cross_num = 0;
	for(uint16_t i = 2;i < BUFFER_SIZE - 2;)
	{
		if((red_buffer[i] < 0) && (red_buffer[i + 1] > 0))
		{
			zero_cross_index[zero_cross_num] = i;
			zero_cross_num++;
			i = i + 2;
		}
		else
		{
			i++;
		}
	}	
	
	if(zero_cross_num < 2)
		return 0;

	
	//计算过零点之间的间距
	for(uint16_t i = 0;i < zero_cross_num - 1;i++)
	{
		zero_cross_index[i] = zero_cross_index[i + 1] - zero_cross_index[i];
	}
	
	//找到合适的间距
	static float old_interval = 0;
	float index_interval = Find_Melia(zero_cross_index,zero_cross_num - 1,30);
	if((index_interval <= 25) || (index_interval >= 150))
	{
		old_interval = 0;
		return 0;
	}
	
	
	if(old_interval == 0)
		old_interval = index_interval;
	
	index_interval = (index_interval + old_interval)/2;
	old_interval = index_interval;
	
	Heart_Rate = 60 * 100.0 /index_interval;
	
	return Heart_Rate;
}

/**********************************************************************
 * @brief 	 		取5个点做中值数滤波
 * @parameter  	buffer 缓冲区指针
								center_index 缓冲区数据索引
 * @return    	float 滤波后的值
  ********************************************************************/
static float Median_Filter(float * buffer,uint16_t center_index)
{
	//取出5个数
	float temp[5];
	for(uint16_t i = 0;i < 5;i++)
	{
		temp[i] = buffer[center_index - 2 + i];
	}
	
	//冒泡排序
	int j,i;
	float tem;
	for (i = 0; i < 5-1;i ++)//size-1是因为不用与自己比较，所以比的数就少一个
	{
		int count = 0;
		for (j = 0; j < 5-1 - i; j++)	//size-1-i是因为每一趟就会少一个数比较
		{
			if (temp[j] > temp[j+1])//这是升序排法，前一个数和后一个数比较，如果前数大则与后一个数换位置
			{
				tem = temp[j];
				temp[j] = temp[j+1];
				temp[j+1] = tem;
				count = 1;
				
			}
		}
		if (count == 0)			//如果某一趟没有交换位置，则说明已经排好序，直接退出循环
				break;	
	}
	
	tem = 0.7*(double)temp[2] + 0.1*(double)temp[1] + 0.1*(double)temp[3] + 0.05*(double)temp[0] + 0.05*(double)temp[4];
	return tem;
}

/**********************************************************************
	* @brief 	 		对缓冲区做中值滤波
 * @parameter  	
 * @return    	
  ********************************************************************/
static void Buffer_Median_Filter(float * buffer)
{

	uint16_t i;
	//处理red数据
	temp_buffer[0] = (buffer[0] + buffer[1])/2;
	temp_buffer[1] = (buffer[1] + buffer[2])/2;	
	for(i = 2;i < BUFFER_SIZE-2;i++) 
	{
			temp_buffer[i] = Median_Filter(buffer,i);			
	}
	temp_buffer[BUFFER_SIZE-2] = (buffer[BUFFER_SIZE-3] + buffer[BUFFER_SIZE-2])/2;
	temp_buffer[BUFFER_SIZE-1] = (buffer[BUFFER_SIZE-2] + buffer[BUFFER_SIZE-1])/2;			
	for(i = 0;i < BUFFER_SIZE;i++)
	{
		buffer[i] = temp_buffer[i];
	}
}
/**********************************************************************
 * @brief 	 		计算血氧
 * @parameter  	
 * @return    	
  ********************************************************************/
static float Calculate_SpO2(float dc_red,float dc_ir,uint16_t cache_nums)
{
	uint16_t i;
	float ac_red,ac_ir;
	
	for(i = 0;i < FFT_N;i++)
	{
		s1[i].real = red_buffer[i];
		s2[i].real = ir_buffer[i];
	}	
	
	//快速傅里叶变换
	FFT(s1);
	FFT(s2);
	
	//解平方
	for(i = 0;i < FFT_N;i++) 
	{
		s1[i].real=sqrtf(s1[i].real*s1[i].real+s1[i].imag*s1[i].imag);
		s1[i].real=sqrtf(s2[i].real*s2[i].real+s2[i].imag*s2[i].imag);
	}
	//计算交流分量
	for (i=1 ; i<FFT_N ; i++ ) 
	{
		ac_red += s1[i].real ;
		ac_ir +=  s2[i].real ;
	}
	
	 
	//这是商家参考代码的心率算法
	//读取峰值点的横坐标
	int s1_max_index = find_max_num_index(s1, 30);
	int s2_max_index = find_max_num_index(s2, 30);
	//心率是最大的频率分量所对应的频率
	Heart_Rate1 = 60.00 * ((100.0 * s1_max_index )/ FFT_N);

	//这是商家参考代码的血氧算法
	float R = (ac_ir*dc_red)/(ac_red*dc_ir);
	float temp = -45.060*(double)R*(double)R+ 30.354 *(double)R + 94.845;
	if(temp > 99)
		spO2_num =99;
	else
		spO2_num = temp;
	
	
	return 	spO2_num;	
}








/*******************************************************************************
 * Extern function 
 ******************************************************************************/



/**********************************************************************
 * @brief 	 		处理心率血氧数据，在缓冲区满了的时候调用本函数
 * @parameter  	
 * @return    	
  ********************************************************************/
uint16_t Blood_Data_Process(void)
{	
	float n_denom;
	uint16_t i;
	
	float dc_red =0; 
	float dc_ir =0;
	
	//获得直流数据
	for (i=0 ; i<BUFFER_SIZE ; i++ ) 
	{
		dc_red += red_buffer[i] ;
		dc_ir +=  ir_buffer[i] ;
		
	}
	dc_red =dc_red/BUFFER_SIZE ;
	dc_ir =dc_ir/BUFFER_SIZE ;
	
	//如果直流没有达到阈值就不用处理了
	if(dc_ir < IR_THRESHOLD)
	{
		spO2_num = 0;
		Heart_Rate = 0;
		Clear_Buffer_State();
		Chang_to_Standby();
		
		return 0;
	}
		
	
	//中位数滤波，去掉极大或极小的噪点
	Buffer_Median_Filter(red_buffer);
	Buffer_Median_Filter(ir_buffer);
	
	//移动平均滤波
	for(i = 1;i < BUFFER_SIZE-1;i++) 
	{
			n_denom= ( red_buffer[i-1] + 2*red_buffer[i] + red_buffer[i+1]);
			red_buffer[i]=  (double)n_denom/4.00; 
			
			n_denom= ( ir_buffer[i-1] + 2*ir_buffer[i] + ir_buffer[i+1]);
			ir_buffer[i]=  (double)n_denom/4.00; 			
	}
	
	
	//获得交流数据
	for (i=0 ; i<BUFFER_SIZE ; i++ )  
	{
		red_buffer[i] =  red_buffer[i] - dc_red ; 
		ir_buffer[i] =  ir_buffer[i] - dc_ir ; 
	}
	
	Buffer_Median_Filter(red_buffer);
	Buffer_Median_Filter(ir_buffer);
	
	//计算血氧饱和度
	Calculate_SpO2(dc_red,dc_ir,BUFFER_SIZE);
	//计算心率
	Heart_Rate_Process();	
	
	//清除缓冲区状态
	Clear_Buffer_State();
	
	//切换到数据显示界面
	Chang_to_Data();
	
	return 0;
}
			
//获取心率
float Get_Heart_Rate(void)
{
	return	Heart_Rate;
}	

//获取血氧饱和度
float Get_SpO2(void)
{
	return	spO2_num;
}	

//获取商家参考代码的心率算法结果
float Get_Heart_Rate1(void)
{
	return	Heart_Rate1;
}	


