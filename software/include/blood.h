#ifndef _BLOOD_H
#define _BLOOD_H

#include "hc32_ddl.h"                 // Device header



#define IR_THRESHOLD 				32000			//IR阈值




float Get_Heart_Rate(void);
float Get_Heart_Rate1(void);
float Get_SpO2(void);
uint16_t Blood_Data_Process(void);


#endif


