//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief BMP085 lPressure sensor driver
///
//  Change (Lint) non void function calls casted to (void), Bmp85.oversampling
//         casted to (uint8_t)
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"

#include "i2c_mems_driver.h"
#include "bmp085_driver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef VAR_STATIC
#define VAR_STATIC static
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef enum {
    START_TEMP_CONVERSION,
    READ_UNCOMPENSATED_TEMP,
    COMPENSATE_TEMP,
    START_PRESS_CONVERSION,
    READ_UNCOMPENSATED_PRESS,
    COMPENSATE_PRESS,
    COMPUTE_ALTITUDE
} ENUM_BMP085_STATUS;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC STRUCT_BMP85 Bmp85;          // BMP085 device data structure
VAR_STATIC uint16_t uiRaw_Temp;
VAR_STATIC int16_t iTemperature;
VAR_STATIC int32_t lRaw_Press, lPressure, lAltitude;
VAR_STATIC ENUM_BMP085_STATUS eBMP085_Status;
VAR_STATIC portTickType Last_Wake_Time;

/*--------------------------------- Prototypes -------------------------------*/

static void Compensate_Temperature(void);
static void Compensate_Pressure(void);
static uint8_t BMP085_Get_Calibration(void);

///----------------------------------------------------------------------------
///
/// \brief   initialize BMP085
/// \param   -
/// \return  result of communication routines
/// \remarks -
///
///----------------------------------------------------------------------------
uint8_t BMP085_Init(void)
{
  uint8_t data, comres = 0;

  /* read Chip Id */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_CHIP_ID__REG, &data);

  Bmp85.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
  Bmp85.oversampling = 3;

  /* read Version register */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_VERSION_REG, &data);

  /* get ML Version */
  Bmp85.ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);

  /* get AL Version */
  Bmp85.al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);

  /* extract bmp085 calibration parameter structure */
  (void)BMP085_Get_Calibration( );

  eBMP085_Status = START_TEMP_CONVERSION;

  return comres;
}


///----------------------------------------------------------------------------
///
/// \brief   BMP084 handler state machine
/// \param   -
/// \return  -
/// \remarks
///
///
///----------------------------------------------------------------------------
void BMP085_Handler(void)
{
  uint8_t uctemp = 0;
  uint8_t data[3];

  switch (eBMP085_Status) {

    case START_TEMP_CONVERSION:     /* start temperature conversion */
        uctemp = I2C_MEMS_Write_Reg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
        Last_Wake_Time = xTaskGetTickCount();
        eBMP085_Status = READ_UNCOMPENSATED_TEMP;
        break;

    case READ_UNCOMPENSATED_TEMP:   /* read uncompensated temperature */
        if (xTaskGetTickCount() > Last_Wake_Time + BMP085_TEMP_CONVERSION_TIME) {
            uctemp  = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 2);
            uiRaw_Temp = (data[0] << 8) | data[1];
            eBMP085_Status = COMPENSATE_TEMP;
        }
        break;

    case COMPENSATE_TEMP:           /* compute compensated temperature */
        Compensate_Temperature();
        eBMP085_Status = START_PRESS_CONVERSION;
        break;

    case START_PRESS_CONVERSION:    /* start pressure conversion */
        uctemp = BMP085_P_MEASURE + (uint8_t)(Bmp85.oversampling << 6);
        (void)I2C_MEMS_Write_Reg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, uctemp);
        Last_Wake_Time = xTaskGetTickCount();
        eBMP085_Status = READ_UNCOMPENSATED_PRESS;
        break;

    case READ_UNCOMPENSATED_PRESS:  /* read uncompensated pressure */
        if (xTaskGetTickCount() > Last_Wake_Time + (2 + (3 << (Bmp85.oversampling)))) {
            uctemp  = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 3);
            lRaw_Press = (((uint32_t) data[0] << 16) |
                          ((uint32_t) data[1] << 8) |
                           (uint32_t) data[2]) >> (8 - Bmp85.oversampling);
            eBMP085_Status = COMPENSATE_PRESS;
        }
        break;

    case COMPENSATE_PRESS:          /* compute compensated pressure */
        Compensate_Pressure();
        eBMP085_Status = COMPUTE_ALTITUDE;
        break;

    case COMPUTE_ALTITUDE:          /* compute altitude from pressure */
        lAltitude =(((745 * (11390 - (lPressure / 10))) / 256 + 46597) * (11390 - (lPressure / 10))) / 65536 - 966;
        eBMP085_Status = START_PRESS_CONVERSION;
        break;

    default:
        break;
  }
}


///----------------------------------------------------------------------------
///
/// \brief   extract calibration parameters from BMP085 memory
/// \param   -
/// \return  result of communication routines
/// \remarks -
///
///----------------------------------------------------------------------------
static uint8_t BMP085_Get_Calibration(void)
{
  uint8_t comres;
  uint8_t data[BMP085_PROM_DATA_LEN];

  comres = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_PROM_START_ADDR, data, BMP085_PROM_DATA_LEN);
  comres = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_PROM_START_ADDR, data, BMP085_PROM_DATA_LEN);

  /* parameters AC1-AC6 */
  Bmp85.calibration.ac1 =  (data[0] << 8) | data[1];
  Bmp85.calibration.ac2 =  (data[2] << 8) | data[3];
  Bmp85.calibration.ac3 =  (data[4] << 8) | data[5];
  Bmp85.calibration.ac4 =  (data[6] << 8) | data[7];
  Bmp85.calibration.ac5 =  (data[8] << 8) | data[9];
  Bmp85.calibration.ac6 = (data[10] << 8) | data[11];

  /* parameters B1,B2 */
  Bmp85.calibration.b1 =  (data[12] << 8) | data[13];
  Bmp85.calibration.b2 =  (data[14] << 8) | data[15];

  /* parameters MB,MC,MD */
  Bmp85.calibration.mb =  (data[16] << 8) | data[17];
  Bmp85.calibration.mc =  (data[18] << 8) | data[19];
  Bmp85.calibration.md =  (data[20] << 8) | data[21];

  return comres;
}


///----------------------------------------------------------------------------
///
/// \brief   compensate temperature
/// \param   -
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
static void Compensate_Temperature(void)
{
    int32_t x1, x2;

    x1 = (((int32_t)uiRaw_Temp - (int32_t)Bmp85.calibration.ac6) * (int32_t) Bmp85.calibration.ac5) >> 15;
    x2 = ((int32_t) Bmp85.calibration.mc << 11) / (x1 + Bmp85.calibration.md);
    Bmp85.param_b5 = x1 + x2;
    iTemperature = ((Bmp85.param_b5 + 8) >> 4);  // iTemperature in 0.1°C
}


///----------------------------------------------------------------------------
///
/// \brief   compensate pressure
/// \param   -
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
static void Compensate_Pressure(void)
{
    int32_t x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    /* calculate B6 */
    b6 = Bmp85.param_b5 - 4000;

    /* calculate B3 */
    x1 = (b6 * b6) >> 12;
    x1 *= Bmp85.calibration.b2;
    x1 >>= 11;
    x2 = (Bmp85.calibration.ac2 * b6);
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)Bmp85.calibration.ac1) * 4 + x3) << Bmp85.oversampling) + 2) >> 2;

    /* calculate B4 */
    x1 = (Bmp85.calibration.ac3 * b6) >> 13;
    x2 = (Bmp85.calibration.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (Bmp85.calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;

    /* calculate B7 */
    b7 = ((uint32_t)(lRaw_Press - b3) * (50000 >> Bmp85.oversampling));
    if (b7 < 0x80000000) {
      lPressure = (b7 << 1) / b4;
    } else {
      lPressure = (b7 / b4) << 1;
    }

    x1 = lPressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (lPressure * SMD500_PARAM_MH) >> 16;
    lPressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;	// lPressure in Pa
}

///----------------------------------------------------------------------------
///
/// \brief   get compensated temperature
/// \param   -
/// \return  temperature in steps of 0.1 deg celsius
/// \remarks -
///
///----------------------------------------------------------------------------
int16_t BMP085_Get_Temperature(void)
{
  return (iTemperature);
}


///----------------------------------------------------------------------------
///
/// \brief   get compensated pressure
/// \param   -
/// \return  pressure in steps of 0.1 Pa
/// \remarks -
///
///----------------------------------------------------------------------------
int32_t BMP085_Get_Pressure(void)
{
   return (lPressure);
}

///----------------------------------------------------------------------------
///
/// \brief   get altitude
/// \param   -
/// \return  altitude in [m]
/// \remarks -
///
///----------------------------------------------------------------------------
int32_t BMP085_Get_Altitude(void)
{
   return (lAltitude);
}

