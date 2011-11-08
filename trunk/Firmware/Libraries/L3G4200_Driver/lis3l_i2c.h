/********************************************************************************
* @file l3g_i2c.h
* @author Peter Lanius
* @version V0.10
* @date 09/12/2009
* @brief This file provides functions to operate the L3GV02DQ 3-axis
         accelerometer in I2C bus mode.
******************************************************************************
* @copy
*/

/* Includes ----------------------------------------------------------*/

#include "stm32f10x.h"

/* Defines for the GPIO pins used for the I2C communication */
#define I2C_L3G I2C1
#define I2C_L3G_CLK RCC_APB1Periph_I2C1
#define I2C_L3G_GPIO GPIOB
#define I2C_L3G_GPIO_CLK RCC_APB2Periph_GPIOB
#define I2C_L3G_SCL GPIO_Pin_6
#define I2C_L3G_SDA GPIO_Pin_7

/* Define a structure for the 16bit Acceleration data */
typedef struct {
    uint16_t X_Accel;
    uint16_t Y_Accel;
    uint16_t Z_Accel;
} L3G_Accel_t;

// Physical Device Address - Factory Assigned to 0x3A for L3GV02DQ
#define L3G_SLAVE_ADDR 0x3A
// The Who Am I register contains the physical device address
#define L3G_WHO_AM_I 0x0F
// Define the Initialisation code as per the data sheet
#define L3G_INIT_CODE 0xC7
// Set CTRL_REG1 to Run Mode, 640Hz data rate and X,Y,Z enabled
#define L3G_RUN_CODE 0xE7

// Control registers
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21

// Output data
// X axis acceleration data LSB
#define L3G_OUTX_L 0x28
// X axis acceleration data MSB
#define L3G_OUTX_H 0x29
// Y axis acceleration data LSB
#define L3G_OUTY_L 0x2A
// Y axis acceleration data MSB
#define L3G_OUTY_H 0x2B
// Z axis acceleration data LSB
#define L3G_OUTZ_L 0x2C
// Z axis acceleration data MSB
#define L3G_OUTZ_H 0x2D

/* Function Definitions ------------------------------------------------------- */

void I2C_L3G_Init( void);
uint8_t I2C_L3G_ReadByte( uint8_t ReadAddr);
void I2C_L3G_ReadAccel (uint8_t ReadAddr, L3G_Accel_t* L3G_AccelStruct) ;
void I2C_L3G_Write( uint8_t WriteAddr, uint8_t DataByte);

