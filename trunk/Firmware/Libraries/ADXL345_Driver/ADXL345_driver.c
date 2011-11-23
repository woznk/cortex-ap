//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief I2C driver for MEMS sensors
//  Change: added function Set_Fifo_Mode(), function GetAccelRaw() returns bool
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
/// \brief   Set ADXL345 output rate and bandwidth
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   rate_code, output rate code
/// \remarks -
///
///----------------------------------------------------------------------------
static status_t Set_Output_Rate( uint8_t rate_code )
{
  uint8_t value;

  if (!ReadReg(ADXL345_SLAVE_ADDR, BW_RATE, &value))
    return MEMS_ERROR;

  value &= 0xF0;                // Clear rate field
  value |= (rate_code & 0x0F);  // New rate value

  if (!WriteReg(ADXL345_SLAVE_ADDR, BW_RATE, value))
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

  if (!ReadReg(ADXL345_SLAVE_ADDR, POWER_CTL, &value))
    return MEMS_ERROR;

  value |= MEASURE;         // Start measure

  if (!WriteReg(ADXL345_SLAVE_ADDR, POWER_CTL, value))
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

  if (!ReadReg(ADXL345_SLAVE_ADDR, FIFO_CTL, &value))
    return MEMS_ERROR;

  value &= 0x3F;            // clear FIFO mode
  value |= (mode & 0xC0);   // set new FIFO mode

  if (!WriteReg(ADXL345_SLAVE_ADDR, FIFO_CTL, value))
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

  if (!ReadReg(ADXL345_SLAVE_ADDR, DEVID, &id))
      id = 0;
  Set_Fifo_Mode(BYPASS_MODE);   // Disable FIFO
  Set_Output_Rate(0x09);        // Set output rate to 100 Hz
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
bool GetAccelRaw(AccelRaw_t* buff) {
  uint8_t valueL;
  uint8_t valueH;

  if (!ReadReg(ADXL345_SLAVE_ADDR, INT_SOURCE, &valueL) )
      return FALSE;

   if ((valueL & DATA_READY) == 0)
      return FALSE;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAX0, &valueL) )
      return FALSE;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAX1, &valueH) )
      return FALSE;

  buff->x = (short int)((valueH << 8) | valueL );

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAY0, &valueL) )
      return FALSE;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAY1, &valueH) )
      return FALSE;

  buff->y = (short int)((valueH << 8) | valueL );

   if (!ReadReg(ADXL345_SLAVE_ADDR, DATAZ0, &valueL) )
      return FALSE;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAZ1, &valueH) )
      return FALSE;

  buff->z = (short int)((valueH << 8) | valueL );

  return TRUE;
}

