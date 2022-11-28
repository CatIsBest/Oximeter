/**
 * @file GUI.h
 *
 */


#ifndef GUI_H
#define GUI_H

/*********************
 *      INCLUDES
 *********************/
#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "..\lvgl\lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/
 

/**********************
 *      TYPEDEFS
 **********************/
 
//要添加APP界面的话在这里加屏幕比如 SCREEN_APP3
typedef enum
{
	SCREEN_STANDBY,
	SCREEN_TEST,
	SCREEN_DATA,
//	SCREEN_APP3,
}Screen_Type;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void Standby_Screen_Init(void);
void Test_Screen_Init(void);
void Data_Screen_Init(void);
void Change_Screen(void);
Screen_Type Get_Current_Screen(void);
void Chang_to_Standby(void);
void Chang_to_Test(void);
void Chang_to_Data(void);
/**********************
 *      MACROS
 **********************/



#endif /*GUI_H*/
