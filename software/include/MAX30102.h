#ifndef MAX30102_H_
#define MAX30102_H_

#include "hc32_ddl.h"


/*******************************************************************************
 * define
 ******************************************************************************/
 
/* Define I2C unit used for the example */
#define I2C_UNIT                        (M4_I2C1)


/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (PortA)
#define I2C_SCL_PIN                     (Pin10)
#define I2C_SDA_PORT                    (PortA)
#define I2C_SDA_PIN                     (Pin11)
#define I2C_GPIO_SCL_FUNC               (Func_I2c1_Scl)
#define I2C_GPIO_SDA_FUNC               (Func_I2c1_Sda)

#define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C1)

#define TIMEOUT                         (0x10000ul)

/* Define i2c baudrate */
#define I2C_BAUDRATE                    (100000ul)




/* Define slave device address (MAX30102) */
#define DEVICE_ADDRESS                  (0x57u)
//#define I2C_10BITS_ADDRESS              (1u)

#define  INT_PORT       (PortH)
#define  INT_PIN        (Pin02)

// MAX30102 register addresses
#define REG_INTR_STATUS_1 	0x00
#define REG_INTR_STATUS_2 	0x01
#define REG_INTR_ENABLE_1 	0x02
#define REG_INTR_ENABLE_2 	0x03
#define REG_FIFO_WR_PTR 		0x04
#define REG_OVF_COUNTER 		0x05
#define REG_FIFO_RD_PTR 		0x06
#define REG_FIFO_DATA 			0x07
#define REG_FIFO_CONFIG 		0x08
#define REG_MODE_CONFIG 		0x09
#define REG_SPO2_CONFIG 		0x0A
#define REG_LED1_PA 				0x0C
#define REG_LED2_PA 				0x0D
#define REG_PILOT_PA 				0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 			0x1F
#define REG_TEMP_FRAC 			0x20
#define REG_TEMP_CONFIG 		0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 					0xFE
#define REG_PART_ID 				0xFF



// MAX30102 data buffer size
#define BUFFER_SIZE 				512



/*******************************************************************************
 * Extern variable
 ******************************************************************************/
extern float red_buffer[BUFFER_SIZE+16];           	
extern float ir_buffer[BUFFER_SIZE+16]; 

/*******************************************************************************
 * function prototypes
 ******************************************************************************/
// IIC
en_result_t I2C_Master_Transmit(uint16_t u16DevAddr, uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut);
en_result_t I2C_Master_Receive(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut);
en_result_t Master_Initialize(void);

// MAX30102
void MAX30102_Init(void);

en_result_t MAX30102_Write_Reg(uint8_t reg,uint8_t data);
en_result_t MAX30102_Read_Reg(uint8_t reg,uint8_t * data);
void Max30102_reset(void);
void Clear_Buffer_State(void);
uint8_t Get_Buffer_State(void);
uint16_t Get_Buffer_Index(void);

#endif	 /* MAX30102_H_ */
