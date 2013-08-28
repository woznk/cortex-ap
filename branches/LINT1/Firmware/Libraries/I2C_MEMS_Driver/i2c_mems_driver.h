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
/// Changes: Added prefix I2C_MEMS_ to all functions
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

#define MEMS_SET                0x01
#define MEMS_RESET              0x00

// Mask to check both RXNE and BTF flags (definitions of ST library don't work)
#define I2C_RXNE_BTF_FLAGS      ((uint32_t)0x00030044)

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef enum {
  MEMS_SUCCESS = 0x01,
  MEMS_ERROR   = 0x00
} status_t;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

uint8_t I2C_MEMS_Read_Reg(uint8_t slave, uint8_t Reg, uint8_t* Data);
uint8_t I2C_MEMS_Write_Reg(uint8_t slave, uint8_t Reg, uint8_t Data);
uint8_t I2C_MEMS_Read_Buff(uint8_t slave, uint8_t reg, uint8_t* data, uint8_t length);
void I2C_MEMS_Init( void );

#endif /* __I2C_MEMS_DRIVER_H */




