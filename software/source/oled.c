
/*******************************************************************************
 * @file 	 		oled.c
 * @author  	日常里的奇迹	@bilibili
 * @date    	2022-11-24
 * @brief			0.96寸 SPI-OLED 驱动 
********************************************************************************/

#include "oled.h"


/*******************************************************************************
 * Local function 
 ******************************************************************************/

//配置GPIO
static void Gpio_Config(void)
{
		stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Disable;
    stcPortInit.enPullUp = Enable;

    PORT_Init(RST_PORT, RST_PIN, &stcPortInit);
		PORT_Init(DC_PORT, DC_PIN, &stcPortInit);
}


//配置SPI
static void Spi_Config(void)
{
    stc_spi_init_t stcSpiInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv64;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelHigh;
    stcSpiInit.enSckPhase = SpiSckOddChangeEvenSample;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode3Line;
    stcSpiInit.enTransMode = SpiTransOnlySend;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;

#ifdef SPI_MASTER_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
#endif

    SPI_Init(SPI_UNIT, &stcSpiInit);
    SPI_Cmd(SPI_UNIT, Enable);
}



/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/


//写命令
void LCD_WrCmd(uint8_t cmd)
{
	/* Wait tx buffer empty */
	while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSendBufferEmpty))
	{
	}	
	LCD_DC_CLR();
	/* Send data */
	SPI_SendData8(SPI_UNIT, cmd);
	Ddl_Delay1us(8);
}

//写数据
void LCD_WrDat(uint8_t data)
{
	/* Wait tx buffer empty */
	while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSendBufferEmpty))
	{
	}
	LCD_DC_SET();
	/* Send data */
	SPI_SendData8(SPI_UNIT, data);
	Ddl_Delay1us(8);
}

//填充指定值
void LCD_Fill(uint8_t bmp_data)
{
	uint8_t y,x;
	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x00);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(bmp_data);
	}
}

//设定位置（y是8的倍数）
void LCD_Set_Pos(uint8_t x, uint8_t y)
{ 
  LCD_WrCmd(0xb0+(y>>3));
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd(x&0x0f); 
} 

//初始化 LCD/OLED
void LCD_Init(void)        
{	
	Spi_Config();
	Gpio_Config();
	
	LCD_RST_CLR();
	Ddl_Delay1ms(50);
	LCD_RST_SET();

  LCD_WrCmd(0xae);//--turn off oled panel
  LCD_WrCmd(0x00);//---set low column address
  LCD_WrCmd(0x10);//---set high column address
  LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  LCD_WrCmd(0x81);//--set contrast control register
  LCD_WrCmd(0xcf); // Set SEG Output Current Brightness
  LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     
  LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   
  LCD_WrCmd(0xa6);//--set normal display
  LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  LCD_WrCmd(0x3f);//--1/64 duty
  LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  LCD_WrCmd(0x00);//-not offset
  LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  LCD_WrCmd(0xd9);//--set pre-charge period
  LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  LCD_WrCmd(0xda);//--set com pins hardware configuration
  LCD_WrCmd(0x12);
  LCD_WrCmd(0xdb);//--set vcomh
  LCD_WrCmd(0x40);//Set VCOM Deselect Level
  LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  LCD_WrCmd(0x02);//
  LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
  LCD_WrCmd(0x14);//--set(0x10) disable
  LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  LCD_WrCmd(0xa7);// Enable Inverse Display On (0xa6/a7) 
  LCD_WrCmd(0xaf);//--turn on oled panel
  LCD_Fill(0xff); //clear the screen
  LCD_Set_Pos(0,0);  
	
	
	//用于测试
//	LCD_WrDat(0xff);
//	LCD_WrDat(0x81);
//	LCD_WrDat(0x81);
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0xff);
//	LCD_WrCmd(0xa5);

//	LCD_Set_Pos(56,24);
//	
//	LCD_WrDat(0xff);
//	LCD_WrDat(0x81);
//	LCD_WrDat(0x81);
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0x81);	
//	LCD_WrDat(0xff);
} 
