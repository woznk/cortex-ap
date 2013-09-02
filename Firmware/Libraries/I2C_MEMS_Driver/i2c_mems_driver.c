//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief I2C driver for MEMS sensors
///
/// Changes: (Lint) local variables declared VAR_STATIC, single line loops
//           replaced with code block, non void function calls casted to (void)
//
//============================================================================*/

#include "cpal_i2c.h"
#include "i2c_mems_driver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef VAR_STATIC
#define VAR_STATIC static
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC CPAL_TransferTypeDef TransferRx;
VAR_STATIC CPAL_TransferTypeDef TransferTx;

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   Reads a register
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, register address
/// \param   *data, pointer to destination data
/// \remarks -
///
///----------------------------------------------------------------------------
uint8_t I2C_MEMS_Read_Reg(uint8_t slave, uint8_t reg, uint8_t* data)
{
	// Wait while the device is busy
    while (CPAL_I2C_IsDeviceReady(&I2C1_DevStructure) != CPAL_PASS){
    }
    I2C1_DevStructure.pCPAL_TransferRx->pbBuffer = data;
    I2C1_DevStructure.pCPAL_TransferRx->wNumData = 1;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr1 = slave;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr2 = reg;
    if (CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS) {
        // Wait end of read operation
        while ((CPAL_StateTypeDef)(I2C1_DevStructure.CPAL_State) != CPAL_STATE_READY){
        }
        return 1;
    } else {
        return 0;
    }
}

///----------------------------------------------------------------------------
///
/// \brief   Read a sequence of registers
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, address of starting register
/// \param   *data, pointer to destination data
/// \param   length, number of registers to read
/// \remarks -
///
///----------------------------------------------------------------------------
uint8_t I2C_MEMS_Read_Buff(uint8_t slave, uint8_t reg, uint8_t* data, uint8_t length)
{
    // Wait while the device is busy
    while (CPAL_I2C_IsDeviceReady(&I2C1_DevStructure) != CPAL_PASS){
    }
    I2C1_DevStructure.pCPAL_TransferRx->pbBuffer = data;
    I2C1_DevStructure.pCPAL_TransferRx->wNumData = length;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr1 = slave;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr2 = reg;
    if (CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS) {
        // Wait end of read operation
        while ((CPAL_StateTypeDef)(I2C1_DevStructure.CPAL_State) != CPAL_STATE_READY){
        }
        return 1;
    } else {
        return 0;
    }
}

///----------------------------------------------------------------------------
///
/// \brief   Write a register
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, register address
/// \param   data, data to be written
/// \remarks see
///
///----------------------------------------------------------------------------
uint8_t I2C_MEMS_Write_Reg(uint8_t slave, uint8_t reg, uint8_t data)
{
    // Wait while the device is busy
    while (CPAL_I2C_IsDeviceReady(&I2C1_DevStructure) != CPAL_PASS){
    }
    I2C1_DevStructure.pCPAL_TransferTx->pbBuffer = &data;
    I2C1_DevStructure.pCPAL_TransferTx->wNumData = 1;
    I2C1_DevStructure.pCPAL_TransferTx->wAddr1 = slave;
    I2C1_DevStructure.pCPAL_TransferTx->wAddr2 = reg;
    if (CPAL_I2C_Write(&I2C1_DevStructure) == CPAL_PASS) {
        // Wait end of read operation
        while ((CPAL_StateTypeDef)(I2C1_DevStructure.CPAL_State) != CPAL_STATE_READY){
        }
        return 1;
    } else {
        return 0;
    }
}


///----------------------------------------------------------------------------
///
/// \brief   Initializes peripherals used by the I2C driver.
/// \return  -
/// \param   -
/// \remarks -
///
///----------------------------------------------------------------------------
void I2C_MEMS_Init( void )
{
  (void)CPAL_I2C_StructInit(&I2C1_DevStructure);    // Initialize CPAL structure for I2C 1

  I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  I2C1_DevStructure.pCPAL_TransferRx = &TransferRx;
  I2C1_DevStructure.pCPAL_TransferTx = &TransferTx;

  (void)CPAL_I2C_Init(&I2C1_DevStructure);          // Initialize CPAL I2C
}
