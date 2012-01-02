//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief I2C driver for MEMS sensors
///
//  Change: Set_Range(): added
//          ADXL345_Init(): full range set to 8 g
//
//============================================================================*/

#include "i2c_mems_driver.h"
#include "adxl345_driver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   Set ADXL345 full scale range
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   range_code, range code
/// \remarks range codes: 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
///
///----------------------------------------------------------------------------
static status_t Set_Range( uint8_t range_code )
{
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, DATA_FORMAT, &value))
    return MEMS_ERROR;

  value &= 0xFC;                // Clear range field
  value |= (range_code & 0x03); // New range value

  if (!I2C_MEMS_Write_Reg(ADXL345_SLAVE_ADDR, DATA_FORMAT, value))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

///----------------------------------------------------------------------------
///
/// \brief   Set ADXL345 output rate and bandwidth
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   rate_code, output rate code
/// \remarks -
///
///----------------------------------------------------------------------------
static status_t Set_Output_Rate( uint8_t rate_code )
{
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, BW_RATE, &value))
    return MEMS_ERROR;

  value &= 0xF0;                // Clear rate field
  value |= (rate_code & 0x0F);  // New rate value

  if (!I2C_MEMS_Write_Reg(ADXL345_SLAVE_ADDR, BW_RATE, value))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

///----------------------------------------------------------------------------
///
/// \brief   Start ADXL345 measurement
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   -
/// \remarks -
///
///----------------------------------------------------------------------------
static status_t Start_Measurement( void )
{
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, POWER_CTL, &value))
    return MEMS_ERROR;

  value |= MEASURE;         // Start measure

  if (!I2C_MEMS_Write_Reg(ADXL345_SLAVE_ADDR, POWER_CTL, value))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

///----------------------------------------------------------------------------
///
/// \brief   Set FIFO mode
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   mode, FIFO mode
/// \remarks -
///
///----------------------------------------------------------------------------
static status_t Set_Fifo_Mode(uint8_t mode)
{
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, FIFO_CTL, &value))
    return MEMS_ERROR;

  value &= 0x3F;            // clear FIFO mode
  value |= (mode & 0xC0);   // set new FIFO mode

  if (!I2C_MEMS_Write_Reg(ADXL345_SLAVE_ADDR, FIFO_CTL, value))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


///----------------------------------------------------------------------------
///
/// \brief   Initialization of ADXL345 accelerometer
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
bool ADXL345_Init( void )
{
  uint8_t id;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, DEVID, &id)) {
      id = 0;
  }
  Set_Fifo_Mode(BYPASS_MODE);   // Disable FIFO
  Set_Range(RANGE_8G);          // Set full scale range to +/- 8 g
  Set_Output_Rate(RATE_100HZ);  // Set output rate to 100 Hz
  Start_Measurement( );         // Start measurement
  return (bool)(id == I_AM_ADXL345);
}

///----------------------------------------------------------------------------
///
/// \brief   Read the acceleration registers
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   *buff, pointer to acceleration structure
/// \remarks -
///
///----------------------------------------------------------------------------
bool GetAccelRaw(uint8_t* data) {
/*
  uint8_t status;

  if (!I2C_MEMS_Read_Reg(ADXL345_SLAVE_ADDR, INT_SOURCE, &status)) {
     return FALSE;
  }

  if ((status & DATA_READY) == 0) {
     return FALSE;
  }
*/
  if (!I2C_MEMS_Read_Buff(ADXL345_SLAVE_ADDR, DATAX0, data, 6)) {
     return FALSE;
  }

  return TRUE;
}

