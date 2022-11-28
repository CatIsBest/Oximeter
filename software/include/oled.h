#ifndef OLED_H_
#define OLED_H_

#include "hc32_ddl.h"

/*******************************************************************************
 * define
 ******************************************************************************/
 
/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    (PortB)
#define SPI_SCK_PIN                     (Pin15)
#define SPI_SCK_FUNC                    (Func_Spi3_Sck)

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortB)
#define SPI_MOSI_PIN                    (Pin14)
#define SPI_MOSI_FUNC                   (Func_Spi3_Mosi)


/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI3)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI3)

/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE


/* RST Port/Pin definition */
#define  RST_PORT        (PortB)
#define  RST_PIN         (Pin13)

/* DC Port/Pin definition */
#define  DC_PORT        (PortB)
#define  DC_PIN         (Pin12)

#define LCD_RST_CLR()	PORT_ResetBits(RST_PORT,RST_PIN)
#define LCD_RST_SET()	PORT_SetBits(RST_PORT,RST_PIN)

#define LCD_DC_CLR()	PORT_ResetBits(DC_PORT,DC_PIN)
#define LCD_DC_SET()	PORT_SetBits(DC_PORT,DC_PIN)

#define X_WIDTH 128
#define Y_WIDTH 64
/*******************************************************************************
 * function prototypes
 ******************************************************************************/
 
void LCD_Init(void);

void LCD_WrCmd(uint8_t cmd);
void LCD_WrDat(uint8_t data);
void LCD_Set_Pos(uint8_t x, uint8_t y);
void LCD_Fill(uint8_t bmp_data); 

#endif	 /* OLED_H_ */
