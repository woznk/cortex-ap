//============================================================================
//
// File name:
// $Revision: $
// $Date: $
// $Author: Lorenz $
//
/// \brief Driver file for I2C MEMS sensors
///
/// \file
///
/// CHANGES
//
//============================================================================

/*------------------------- Prevent recursive inclusion ----------------------*/

#ifndef __I2C_MEMS_DRIVER_H
#define __I2C_MEMS_DRIVER_H

#include "stm32f10x.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

// Defines for the GPIO pins used for the I2C communication
#define I2C_MEMS                I2C1
#define I2C_MEMS_CLK            RCC_APB1Periph_I2C1
#define I2C_MEMS_GPIO           GPIOB
#define I2C_MEMS_GPIO_CLK       RCC_APB2Periph_GPIOB
#define I2C_MEMS_SCL            GPIO_Pin_6
#define I2C_MEMS_SDA            GPIO_Pin_7

#define I2C_MEMS_Speed          100000  // 100kHz bus speed (up to 400kHz is ok)
#define I2C_SLAVE_ADDRESS7      0xA0    // I2C own address if in slave mode

//#define BIT(x) ( 1<<(x) )
#define BIT(x) ( (x) )
#define MEMS_SET                0x01
#define MEMS_RESET              0x00

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

uint8_t ReadReg(uint8_t slave, uint8_t Reg, uint8_t* Data);
uint8_t WriteReg(uint8_t slave, uint8_t Reg, uint8_t Data);
void I2C_MEMS_Init( void );

#endif /* __I2C_MEMS_DRIVER_H */




