//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief I2C driver for MEMS sensors
//  Change: Added function GetAccelRaw(), filled function ADXL345_Init()
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
/// \brief   Initialization of ADXL345 accelerometer
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void ADXL345_Init( void )
{
  Set_Output_Rate(0x09);    // Set Output Rate to 100 Hz
  Start_Measurement( );     // Start measurement
}

///----------------------------------------------------------------------------
///
/// \brief   Read the acceleration registers
/// \return  MEMS_SUCCESS / MEMS_ERROR
/// \param   *buff, pointer to acceleration structure
/// \remarks -
///
///----------------------------------------------------------------------------
status_t GetAccelRaw(AccelRaw_t* buff) {
  unsigned char valueL;
  unsigned char valueH;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAX0, &valueL) )
      return MEMS_ERROR;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAX1, &valueH) )
      return MEMS_ERROR;

  buff->x = (short int)((valueH << 8) | valueL );

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAY0, &valueL) )
      return MEMS_ERROR;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAY1, &valueH) )
      return MEMS_ERROR;

  buff->y = (short int)((valueH << 8) | valueL );

   if (!ReadReg(ADXL345_SLAVE_ADDR, DATAZ0, &valueL) )
      return MEMS_ERROR;

  if (!ReadReg(ADXL345_SLAVE_ADDR, DATAZ1, &valueH) )
      return MEMS_ERROR;

  buff->z = (short int)((valueH << 8) | valueL );

  return MEMS_SUCCESS;
}
