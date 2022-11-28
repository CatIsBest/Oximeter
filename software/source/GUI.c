/*******************************************************************************
 * @file 	 		GUI.c
 * @author  	日常里的奇迹	@bilibili
 * @date    	2022-11-27
 * @brief			用户界面
********************************************************************************/



/*********************
 *      INCLUDES
 *********************/
#include "GUI.h"

#include "oled.h"
#include "MAX30102.h"
#include "blood.h"
/*********************
 *      DEFINES
 *********************/



/**********************
 *  STATIC PROTOTYPES
 **********************/



static struct{
	Screen_Type current_screen;
	Screen_Type next_screen;
	uint8_t lock;
}screen_state = {SCREEN_STANDBY,SCREEN_STANDBY,0};

//// 待机 屏幕
//static lv_obj_t * standby_scr;
//// 数据测试中 屏幕
//static lv_obj_t * test_scr;
//// 数据 屏幕
//static lv_obj_t * data_scr;
////// APP3 屏幕
////static lv_obj_t * app3_scr;
////...


// 数据测试中 屏幕 组件

// 数据 屏幕 组件
static lv_obj_t* label_HR;
static lv_obj_t* label_SpO2;
//// APP3 屏幕 组件
//
//...

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/


LV_IMG_DECLARE(walnut1)
LV_IMG_DECLARE(walnut2)
LV_IMG_DECLARE(walnut3)
LV_IMG_DECLARE(walnut4)
LV_IMG_DECLARE(walnut5)
LV_IMG_DECLARE(walnut6)
LV_IMG_DECLARE(walnut7)
LV_IMG_DECLARE(walnut8)
LV_IMG_DECLARE(walnut9)
LV_IMG_DECLARE(walnut10)
LV_IMG_DECLARE(walnut11)
LV_IMG_DECLARE(walnut12)
LV_IMG_DECLARE(walnut13)
LV_IMG_DECLARE(walnut14)
LV_IMG_DECLARE(walnut15)
LV_IMG_DECLARE(walnut16)
LV_IMG_DECLARE(walnut17)
LV_IMG_DECLARE(walnut18)
LV_IMG_DECLARE(walnut19)
LV_IMG_DECLARE(walnut20)
LV_IMG_DECLARE(walnut21)
LV_IMG_DECLARE(walnut22)
LV_IMG_DECLARE(walnut23)
LV_IMG_DECLARE(walnut24)
LV_IMG_DECLARE(walnut25)
LV_IMG_DECLARE(walnut26)
LV_IMG_DECLARE(walnut27)
LV_IMG_DECLARE(walnut28)
LV_IMG_DECLARE(walnut29)
LV_IMG_DECLARE(walnut30)
LV_IMG_DECLARE(walnut31)
LV_IMG_DECLARE(walnut32)
LV_IMG_DECLARE(walnut33)
LV_IMG_DECLARE(walnut34)
LV_IMG_DECLARE(walnut35)
LV_IMG_DECLARE(walnut36)


static const lv_img_dsc_t* anim_imgs[36] = {
		&walnut1,
		&walnut2,
		&walnut3,
		&walnut4,
		&walnut5,
		&walnut6,
		&walnut7,
		&walnut8,
		&walnut9,
		&walnut10,
		&walnut11,
		&walnut12,
		&walnut13,
		&walnut14,
		&walnut15,
		&walnut16,
		&walnut17,
		&walnut18,
		&walnut19,
		&walnut20,
		&walnut21,
		&walnut22,
		&walnut23,
		&walnut24,
		&walnut25,
		&walnut26,
		&walnut27,
		&walnut28,
		&walnut29,
		&walnut30,
		&walnut31,
		&walnut32,
		&walnut33,
		&walnut34,
		&walnut35,
		&walnut36,
};
 






/**
  * @brief  切换界面 并处理当前界面的组件变动
  */
void Change_Screen(void)
{
	if(screen_state.lock == 0)
	{
		screen_state.lock = 1;
		
		if(screen_state.next_screen != screen_state.current_screen)
		{
			//加载新屏幕
			if(screen_state.next_screen == SCREEN_STANDBY)
			{
				Standby_Screen_Init();
			}
			else if(screen_state.next_screen == SCREEN_TEST)
			{
				Test_Screen_Init();			
			}
			else if(screen_state.next_screen == SCREEN_DATA)
			{
				Data_Screen_Init();			
			}
			
			screen_state.current_screen = screen_state.next_screen;
		}
		
		//处理当前屏幕的活动
		if(screen_state.current_screen == SCREEN_STANDBY)
		{
											//Do nothing
		}		
		else if(screen_state.current_screen == SCREEN_TEST)
		{
											//Do nothing
		}
		else if(screen_state.current_screen == SCREEN_DATA)
		{
			uint16_t HR = Get_Heart_Rate();
//			uint16_t HR1 = Get_Heart_Rate1();
			uint16_t SpO2 = Get_SpO2();
			
			
			lv_label_set_text_fmt(label_HR, "HR:%d", HR);	
			lv_label_set_text_fmt(label_SpO2, "SpO2:%d%%", SpO2);
		}
		
		screen_state.lock = 0;
	}
}

/**
  * @brief  初始化 待机 屏幕
  */
void Standby_Screen_Init(void)
{
	lv_obj_t * new_scr = lv_obj_create(NULL);
	
	
	//设置待机动画
	lv_obj_t * animimg0 = lv_animimg_create(new_scr);
	lv_obj_center(animimg0);
	lv_animimg_set_src(animimg0, (lv_img_dsc_t**) anim_imgs, 36);
	lv_animimg_set_duration(animimg0, 1800);
	lv_animimg_set_repeat_count(animimg0, LV_ANIM_REPEAT_INFINITE);
	lv_animimg_start(animimg0);
	
	lv_scr_load_anim(new_scr,LV_SCR_LOAD_ANIM_NONE,0,0,true);
}

/**
  * @brief  初始化 数据测试中 屏幕
  */
void Test_Screen_Init(void)
{
	lv_obj_t * new_scr = lv_obj_create(NULL);	
	
	lv_obj_t * label_Wating = lv_label_create(new_scr);
	lv_label_set_text(label_Wating,"measuring...");
	lv_obj_center(label_Wating);
	
	lv_scr_load_anim(new_scr,LV_SCR_LOAD_ANIM_NONE,0,0,true);
}

/**
  * @brief  初始化 数据 屏幕
  */

void Data_Screen_Init(void)
{
	lv_obj_t * new_scr = lv_obj_create(NULL);
	
	label_HR = lv_label_create(new_scr);
	lv_obj_align(label_HR,LV_ALIGN_TOP_LEFT,0,0);
	
	label_SpO2 = lv_label_create(new_scr);
	lv_obj_align(label_SpO2,LV_ALIGN_LEFT_MID,0,0);
	
	lv_scr_load_anim(new_scr,LV_SCR_LOAD_ANIM_NONE,0,0,true);
}

//切换到待机界面
void Chang_to_Standby(void)
{
		if(screen_state.lock ==  0)
		{
			screen_state.lock = 1;
			screen_state.next_screen = SCREEN_STANDBY;
			screen_state.lock = 0;
		}
}

//切换到数据采集界面
void Chang_to_Test(void)
{
	if(screen_state.lock ==  0)
	{
		screen_state.lock = 1;
		screen_state.next_screen = SCREEN_TEST;
		screen_state.lock = 0;
	}

}

//切换到数据显示界面
void Chang_to_Data(void)
{
	if(screen_state.lock ==  0)
	{
		screen_state.lock = 1;
		screen_state.next_screen = SCREEN_DATA;
		screen_state.lock = 0;
	}
}

//获取当前界面
Screen_Type Get_Current_Screen(void)
{
	return screen_state.current_screen;
}
