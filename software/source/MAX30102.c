
/*******************************************************************************
 * @file 	 		MAX30102.c
 * @author  	日常里的奇迹	@bilibili
 * @date    	2022-11-24
 * @brief			MAX30102 驱动 
********************************************************************************/

#include "MAX30102.h"
#include "math.h"
#include "GUI.h"

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

// 用于存放 MAX30102 数据的缓冲区
float red_buffer[BUFFER_SIZE+16];           	
float ir_buffer[BUFFER_SIZE+16]; 


/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

// 用于标志 MAX30102 数据的缓冲区是否填满，填满了才会进行数据处理
static uint8_t buffer_is_full = 0;

//  MAX30102 数据的缓冲区的索引
static uint16_t index = 0;

/************************************   IIC    ************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 
 
/**
 ******************************************************************************
 ** \brief  Master transmit data
 **
 ** \param  u16DevAddr            The slave address
 ** \param  pu8TxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
en_result_t I2C_Master_Transmit(uint16_t u16DevAddr, uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    I2C_Cmd(I2C_UNIT, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {

#ifdef I2C_10BITS_ADDRESS
        enRet = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2CDirTrans, u32TimeOut);
#else
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u16DevAddr, I2CDirTrans, u32TimeOut);
#endif

        if(Ok == enRet)
        {
            enRet = I2C_TransData(I2C_UNIT, pu8TxData, u32Size,u32TimeOut);
        }
    }

    I2C_Stop(I2C_UNIT,u32TimeOut);
    I2C_Cmd(I2C_UNIT, Disable);

    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Master receive data
 **
 ** \param  u16DevAddr            The slave address
 ** \param  pu8RxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
en_result_t I2C_Master_Receive(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;

    I2C_Cmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {
        if(1ul == u32Size)
        {
            I2C_AckConfig(I2C_UNIT, I2c_NACK);
        }

#ifdef I2C_10BITS_ADDRESS
        enRet = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2CDirReceive, u32TimeOut);
#else
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u16DevAddr, I2CDirReceive, u32TimeOut);
#endif

        if(Ok == enRet)
        {

            enRet = I2C_MasterDataReceiveAndStop(I2C_UNIT, pu8RxData, u32Size, u32TimeOut);
        }

        I2C_AckConfig(I2C_UNIT, I2c_ACK);
    }

    if(Ok != enRet)
    {
        I2C_Stop(I2C_UNIT,u32TimeOut);
    }
    I2C_Cmd(I2C_UNIT, Disable);
    return enRet;
}

/**
 ******************************************************************************
 ** \brief   Initialize the I2C peripheral for master
 ** \param   None
 ** \retval en_result_t                Enumeration value:
 **          - Ok:                     Success
 **          - ErrorInvalidParameter:  Invalid parameter
 ******************************************************************************/
en_result_t Master_Initialize(void)
{
	  /* Initialize I2C port*/
    PORT_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(I2C_FCG_USE, Enable);
	
    en_result_t enRet;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 0ul;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(I2C_UNIT, Enable);

    return enRet;
}



/************************************   MAX30102    ************************************/


/*******************************************************************************
 * Local function 
 ******************************************************************************/


/**********************************************************************
 * @brief 	 		从 MAX30102 读一帧数据，存放到本文件的数据缓冲区，供中断服务调用
 * @parameter  	
 * @return    	
  ********************************************************************/
static void MAX30102_Read_Fream(void)
{
	uint8_t reg_temp;
  
  //read and clear status register
  MAX30102_Read_Reg(REG_INTR_STATUS_1,&reg_temp);
	
	if((reg_temp & 0x10) && (Get_Current_Screen() == SCREEN_STANDBY) && (!buffer_is_full))
	{
		Chang_to_Test();
	}
	
  if(reg_temp & 0x40)
	{
		uint16_t un_temp,red_data,ir_data;
		uint8_t ach_i2c_data[6];
		
		//read a frame
		uint8_t address =REG_FIFO_DATA;
		I2C_Master_Transmit(DEVICE_ADDRESS,&address,1,TIMEOUT);
		I2C_Master_Receive(DEVICE_ADDRESS,ach_i2c_data,6,TIMEOUT);
		
		if((Get_Current_Screen() != SCREEN_STANDBY) && (!buffer_is_full))
		{
			un_temp=ach_i2c_data[0];
			un_temp<<=14;
			red_data+=un_temp;
			un_temp=ach_i2c_data[1];
			un_temp<<=6;
			red_data+=un_temp;
			un_temp=ach_i2c_data[2];
			un_temp>>=2;
			red_data+=un_temp;
			
			un_temp=ach_i2c_data[3];
			un_temp<<=14;
			ir_data+=un_temp;
			un_temp=ach_i2c_data[4];
			un_temp<<=6;
			ir_data+=un_temp;
			un_temp=ach_i2c_data[5];
			un_temp>>=2;
			ir_data+=un_temp;
			
			red_buffer[index] = red_data;
			ir_buffer[index] = ir_data;			
			
			//buffer is full
			if(index >= BUFFER_SIZE - 1)
			{
				index = 0;
				buffer_is_full = 1;
			}
			else
			{
				index++;
			}
		}
	}
}






/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**********************************************************************
 * @brief 	 		写 MAX30102 寄存器
 * @parameter  	reg 寄存器地址
								data 数据
 * @return    	Ok 成功
								ErrorTimeout 超时
  ********************************************************************/
en_result_t MAX30102_Write_Reg(uint8_t reg,uint8_t data)
{
	uint8_t a[2];
	a[0] = reg;
	a[1] = data;
	return I2C_Master_Transmit(DEVICE_ADDRESS,a,2,TIMEOUT);
}

/**********************************************************************
 * @brief 	 		读 MAX30102 寄存器
 * @parameter  	reg 寄存器地址
								data 数据
 * @return    	Ok 成功
								ErrorTimeout 超时
  ********************************************************************/
en_result_t MAX30102_Read_Reg(uint8_t reg,uint8_t * data)
{
	uint8_t a = reg;
	I2C_Master_Transmit(DEVICE_ADDRESS,&a,1,TIMEOUT);
	
	return I2C_Master_Receive(DEVICE_ADDRESS,data,1,TIMEOUT);
}

/**********************************************************************
 * @brief 	 		获取 MAX30102 数据缓冲区状态
 * @parameter  	
 * @return      0	缓冲区没满
                非0 缓冲区满了
  ********************************************************************/
uint8_t Get_Buffer_State(void)
{
	return	buffer_is_full;
}	

/**********************************************************************
 * @brief 	 		把缓冲区状态清零
 * @parameter  	
 * @return    	
  ********************************************************************/
void Clear_Buffer_State(void)
{
	buffer_is_full = 0;
}	

/**********************************************************************
 * @brief 	 		获取缓冲区索引
 * @parameter  	
 * @return    	
  ********************************************************************/
uint16_t Get_Buffer_Index(void)
{
	return index;
}	

/**********************************************************************
 * @brief 	 		MAX30102 复位
 * @parameter  	
 * @return    	
  ********************************************************************/
void Max30102_reset(void)
{
	MAX30102_Write_Reg(REG_MODE_CONFIG,0x40);
}



/**********************************************************************
 * @brief 	 		MAX30102 中断服务
 * @parameter  	
 * @return    	
  ********************************************************************/
void ExtInt02_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh02))
    {
        
			 MAX30102_Read_Fream();
			
       /* clear int request flag */
       EXINT_IrqFlgClr(ExtiCh02);
    }
}

/**********************************************************************
 * @brief 	 		MAX30102 中断初始化
 * @parameter  	
 * @return    	
  ********************************************************************/
static void MAX30102_Int_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /**************************************************************************/
    /* External Int Ch.1                                                      */
    /**************************************************************************/
    stcExtiConfig.enExitCh = ExtiCh02;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntLowLevel;
    EXINT_Init(&stcExtiConfig);

    /* Set External Int */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(INT_PORT, INT_PIN, &stcPortInit);

    /* Select External Int Ch.1 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ2;

    /* Register External Int to Vect.No.000 */
    stcIrqRegiConf.enIRQn = Int000_IRQn;

    /* Callback function */
    stcIrqRegiConf.pfnCallback = &ExtInt02_Callback;

    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);

    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);

    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}


/**********************************************************************
 * @brief 	 		MAX30102 初始化
 * @parameter  	
 * @return    	
  ********************************************************************/
void MAX30102_Init(void)
{
	//Initialize IIC
	Master_Initialize();
	//Initialize interrupt
	MAX30102_Int_Init();
	
	Max30102_reset();
	
	//Initialize MAX30102
	MAX30102_Write_Reg(REG_INTR_ENABLE_1,0xd0);//// INTR setting
	MAX30102_Write_Reg(REG_INTR_ENABLE_2,0x00);//
	MAX30102_Write_Reg(REG_FIFO_WR_PTR,0x00);//FIFO_WR_PTR[4:0]
	MAX30102_Write_Reg(REG_OVF_COUNTER,0x00);//OVF_COUNTER[4:0]
	MAX30102_Write_Reg(REG_FIFO_RD_PTR,0x00);//FIFO_RD_PTR[4:0]
	
	MAX30102_Write_Reg(REG_FIFO_CONFIG,0x0f);//sample avg = 1, fifo rollover=false, fifo almost full = 17
	MAX30102_Write_Reg(REG_MODE_CONFIG,0x03);//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
	MAX30102_Write_Reg(REG_SPO2_CONFIG,0x27);	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
	MAX30102_Write_Reg(REG_LED1_PA,0x32);//Choose value for ~ 10mA for LED1
	MAX30102_Write_Reg(REG_LED2_PA,0x32);// Choose value for ~ 10mA for LED2
	MAX30102_Write_Reg(REG_PILOT_PA,0x7f);// Choose value for ~ 25mA for Pilot LED
	
//	MAX30102_Write_Reg(REG_TEMP_CONFIG,0x01);// Enable Die Temperature
	MAX30102_Write_Reg(REG_PROX_INT_THRESH,0x80);// Set Proximity Interrupt Threshold

}
