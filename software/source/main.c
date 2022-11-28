/*******************************************************************************
 * @file 	 		main.c
 * @author  	日常里的奇迹	@bilibili
 * @date    	2022-11-24
********************************************************************************/


/*******************************************************************************
 * Include files
 ******************************************************************************/
 /* ddl库 */
#include "hc32_ddl.h"


/* BSP */
#include "oled.h"
#include "MAX30102.h"


/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/* LVGL*/
#include "..\lvgl\lvgl.h"
#include "..\lvgl\examples\porting\lv_port_disp_template.h"

/* Application*/
#include "blood.h"
#include "GUI.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
 


/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
 


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
 
static void SysClkConfig(void);/* 时钟配置 */
static void Wdt_Config(void);/* 看门狗配置 */


//FreeRTOS 任务
static void AppTaskCreate(void);/* 创建其他任务 */
static void WDT_Task(void* pvParameters);
static void Display_Task(void* pvParameters);


/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

//FreeRTOS 任务句柄
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t WDT_Task_Handle = NULL;
static TaskHandle_t Display_Task_Handle = NULL;

 
 
/**
 *******************************************************************************
 ** \brief  Main function
 ** \param  None
 ** \retval int32_t Return value, if needed
 ******************************************************************************/
int32_t main(void)
{	
	
	//初始化
	SysClkConfig();		
	SysTick_Init(1000);
	MAX30102_Init();
	
	lv_init();
	lv_port_disp_init();
	
	BaseType_t xReturn = pdPASS;
	
	//创建任务
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate, 
                        (const char*    )"AppTaskCreate",
                        (uint16_t       )512,  
                        (void*          )NULL,
                        (UBaseType_t    )1, 
                        (TaskHandle_t*  )&AppTaskCreate_Handle);
         
  if(pdPASS == xReturn)
    vTaskStartScheduler();   
  else
    return -1;  
	
	while(1)
	{		
		
	}
}

/**********************************************************************
 * @brief 	 		创建任务
 * @parameter  	void
 * @return    	void
  ********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;
  
  taskENTER_CRITICAL();           //进入临界区
	
	//创建看门狗任务
  xReturn = xTaskCreate((TaskFunction_t )WDT_Task, 
                        (const char*    )"WDT_Task",
                        (uint16_t       )256,   
                        (void*          )NULL,	
                        (UBaseType_t    )31,	   
                        (TaskHandle_t*  )&WDT_Task_Handle);

	//创建显示任务
  xReturn = xTaskCreate((TaskFunction_t )Display_Task,
                        (const char*    )"Display_Task",
                        (uint16_t       )1024,  
                        (void*          )NULL,	
                        (UBaseType_t    )4,	    
                        (TaskHandle_t*  )&Display_Task_Handle);				
											
																						
	(void) xReturn;											
																								
  vTaskDelete(AppTaskCreate_Handle); //删除本任务
  
  taskEXIT_CRITICAL();            //退出临界区
}

/**********************************************************************
 * @brief 	 	看门狗任务，用于配置看门狗和喂狗
 * @parameter  	void* parameter 创建任务的时候传进来
 * @return    	void
  ********************************************************************/
static void WDT_Task(void* parameter)
{		
	Wdt_Config();
	WDT_RefreshCounter();
	while (1)
	{			
		WDT_RefreshCounter();																			
		vTaskDelay(1000);  
	}
}

/**********************************************************************
 * @brief 	 		显示任务
 * @parameter  	void* parameter 创建任务的时候传进来
 * @return    	void
  ********************************************************************/
static void Display_Task(void* parameter)
{		

	
  Standby_Screen_Init();
	
	while (1)
	{	
		if(Get_Buffer_State())
		{
			Blood_Data_Process();			
		}
		
		Change_Screen();
		
		lv_timer_handler();
		vTaskDelay(10); 
				
	}
}


/**********************************************************************
 * @brief 	 		配置时钟
 * @parameter  	void* parameter 创建任务的时候传进来
 * @return    	void
  ********************************************************************/
static void SysClkConfig(void)
{
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_sram_config_t       stcSramConfig;
	
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
		MEM_ZERO_STRUCT(stcSramConfig);
	
    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode        = ClkXtalModeOsc;
    stcXtalCfg.enDrv         = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1ul;
    stcMpllCfg.plln    = 42ul;
    stcMpllCfg.PllpDiv = 2ul;
    stcMpllCfg.PllqDiv = 2ul;
    stcMpllCfg.PllrDiv = 2ul;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(5ul);
    EFM_Lock();
		
		/* sram init include read/write wait cycle setting */
    stcSramConfig.u8SramIdx = Sram12Idx | Sram3Idx | SramHsIdx | SramRetIdx;
    stcSramConfig.enSramRC = SramCycle2;
    stcSramConfig.enSramWC = SramCycle2;
    SRAM_Init(&stcSramConfig);

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
        ;
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
		
}

/**********************************************************************
 * @brief 	 		配置看门狗 3.2s溢出 @Pclk3 = 42MHz
 * @parameter  	void* parameter 创建任务的时候传进来
 * @return    	void
  ********************************************************************/
static void Wdt_Config(void)
{
    stc_wdt_init_t stcWdtInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcWdtInit);

    stcWdtInit.enClkDiv = WdtPclk3Div2048;
    stcWdtInit.enCountCycle = WdtCountCycle65536;
    stcWdtInit.enRefreshRange = WdtRefresh100Pct;
    stcWdtInit.enSleepModeCountEn = Disable;
    stcWdtInit.enRequestType = WdtTriggerResetRequest;
    WDT_Init(&stcWdtInit);
}


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
