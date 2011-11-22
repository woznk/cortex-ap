/*******************************************************************************
* File Name:
* $Revision:$
* $Date:$
* driver file
* Change:
*
********************************************************************************/

#include "i2c_mems_driver.h"
#include "adxl345_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void ADXL345_Init( void )
{

  // Configure ADXL345 registers

}

status_t Get_Output_Rate( uint16_t *rate_in_hz )
{
  uint8_t value;

  if (!ReadReg(ADXL345_SLAVE_ADDR, BW_RATE, &value))
    return MEMS_ERROR;

  value &= 0x0F;
  switch (value) {
    case 0x06:
        rate_in_hz = 6;
    break;
    case 0x07:
        rate_in_hz = 12;
    break;
    case 0x08:
        rate_in_hz = 25;
    break;
    case 0x09:
        rate_in_hz = 50;
    break;
    case 0x0A:
        rate_in_hz = 100;
    break;
    case 0x0B:
        rate_in_hz = 200;
    break;
    case 0x0C:
        rate_in_hz = 400;
    break;
    case 0x0D:
        rate_in_hz = 800;
    break;
    case 0x0E:
        rate_in_hz = 1600;
    break;
    case 0x0F:
        rate_in_hz = 3200;
    break;
    default:
        return MEMS_ERROR;
    break;
  }
  return MEMS_SUCCESS;
}

status_t Set_Output_Rate( uint8_t rate_code )
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

status_t Start_Measurement( uint8_t rate )
{
  uint8_t value;

  if (!ReadReg(ADXL345_SLAVE_ADDR, POWER_CTL, &value))
    return MEMS_ERROR;

  value |= MEASURE;         // Start measure

  if (!WriteReg(ADXL345_SLAVE_ADDR, POWER_CTL, value))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}
