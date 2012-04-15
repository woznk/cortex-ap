//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief BMP085 lPressure sensor driver
///
//  Change added BMP085_Handler()
//         renamed variables
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"

#include "i2c_mems_driver.h"
#include "bmp085_driver.h"

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

VAR_STATIC STRUCT_BMP85 Bmp85;      // BMP085 device data structure
VAR_STATIC uint16_t raw_t, raw_p;
VAR_STATIC uint8_t data[3];
VAR_STATIC int16_t iTemperature;
VAR_STATIC int32_t lPressure;
VAR_STATIC ENUM_BMP085_STATUS ucBMP085_Status;
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
  uint8_t comres = 0;

  /* read Chip Id */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_CHIP_ID__REG, data);

  Bmp85.chip_id = BMP085_GET_BITSLICE(data[0], BMP085_CHIP_ID);
  Bmp85.number_of_samples = 1;
  Bmp85.oversampling = 0;

  /* read Version register */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_VERSION_REG, data);

  /* get ML Version */
  Bmp85.ml_version = BMP085_GET_BITSLICE(data[0], BMP085_ML_VERSION);

  /* get AL Version */
  Bmp85.al_version = BMP085_GET_BITSLICE(data[0], BMP085_AL_VERSION);

  /* extract bmp085 calibration parameter structure */
  BMP085_Get_Calibration( );

  ucBMP085_Status = START_TEMP_CONVERSION;

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
  uint8_t ucTemp = 0;

  switch (ucBMP085_Status) {

    case START_TEMP_CONVERSION:     /* start temperature conversion */
        ucTemp = I2C_MEMS_Write_Reg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
        Last_Wake_Time = xTaskGetTickCount();
        ucBMP085_Status = READ_UNCOMPENSATED_TEMP;
        break;

    case READ_UNCOMPENSATED_TEMP:   /* read uncompensated temperature */
        if (xTaskGetTickCount() > Last_Wake_Time + BMP085_TEMP_CONVERSION_TIME) {
            ucTemp += I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 2);
            raw_t = (data[0] << 8) | data[1];
            ucBMP085_Status = COMPENSATE_TEMP;
        }
        break;

    case COMPENSATE_TEMP:           /* compute compensated temperature */
        Compensate_Temperature();
        ucBMP085_Status = START_PRESS_CONVERSION;
        break;

    case START_PRESS_CONVERSION:    /* start pressure conversion */
        ucTemp = BMP085_P_MEASURE + (Bmp85.oversampling << 6);
        I2C_MEMS_Write_Reg(Bmp85.dev_addr, BMP085_CTRL_MEAS_REG, ucTemp);
        Last_Wake_Time = xTaskGetTickCount();
        ucBMP085_Status = READ_UNCOMPENSATED_TEMP;
        break;

    case READ_UNCOMPENSATED_PRESS:  /* read uncompensated pressure */
        if (xTaskGetTickCount() > Last_Wake_Time + (2 + (3 << (Bmp85.oversampling)))) {
            ucTemp += I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 3);
            raw_p = (((uint32_t) data[0] << 16) |
                     ((uint32_t) data[1] << 8) |
                      (uint32_t) data[2]) >> (8 - Bmp85.oversampling);
            Bmp85.number_of_samples = 1;
            ucBMP085_Status = COMPENSATE_PRESS;
        }
        break;

    case COMPENSATE_PRESS:          /* compute compensated pressure */
        Compensate_Pressure();
        ucBMP085_Status = COMPUTE_ALTITUDE;
        break;

    case COMPUTE_ALTITUDE:          /* compute altitude from pressure */
        ucBMP085_Status = START_PRESS_CONVERSION;
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
  unsigned char data[22];

  comres = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR,
                              BMP085_PROM_START_ADDR,
                              data,
                              BMP085_PROM_DATA_LEN);

  /* parameters AC1-AC6 */
  Bmp85.cal_param.ac1 =  (data[0] <<8) | data[1];
  Bmp85.cal_param.ac2 =  (data[2] <<8) | data[3];
  Bmp85.cal_param.ac3 =  (data[4] <<8) | data[5];
  Bmp85.cal_param.ac4 =  (data[6] <<8) | data[7];
  Bmp85.cal_param.ac5 =  (data[8] <<8) | data[9];
  Bmp85.cal_param.ac6 = (data[10] <<8) | data[11];

  /* parameters B1,B2 */
  Bmp85.cal_param.b1 =  (data[12] <<8) | data[13];
  Bmp85.cal_param.b2 =  (data[14] <<8) | data[15];

  /* parameters MB,MC,MD */
  Bmp85.cal_param.mb =  (data[16] <<8) | data[17];
  Bmp85.cal_param.mc =  (data[18] <<8) | data[19];
  Bmp85.cal_param.md =  (data[20] <<8) | data[21];

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

    x1 = (((int32_t) raw_t - (int32_t) Bmp85.cal_param.ac6) * (int32_t) Bmp85.cal_param.ac5) >> 15;
    x2 = ((int32_t) Bmp85.cal_param.mc << 11) / (x1 + Bmp85.cal_param.md);
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
    int32_t x1, x2, x3, b3, b4, b6, b7;

    /* calculate B6 */
    b6 = Bmp85.param_b5 - 4000;

    /* calculate B3 */
    x1 = (b6 * b6) >> 12;
    x1 *= Bmp85.cal_param.b2;
    x1 >>= 11;
    x2 = (Bmp85.cal_param.ac2 * b6);
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)Bmp85.cal_param.ac1) * 4 + x3) << Bmp85.oversampling) + 2) >> 2;

    /* calculate B4 */
    x1 = (Bmp85.cal_param.ac3 * b6) >> 13;
    x2 = (Bmp85.cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (Bmp85.cal_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;

    /* calculate B7 */
    b7 = ((uint32_t)(raw_p - b3) * (50000 >> Bmp85.oversampling));
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
/// \brief   get compensated pPressure
/// \param   -
/// \return  pressure in steps of 0.1 Pa
/// \remarks -
///
///----------------------------------------------------------------------------
int32_t BMP085_Get_Pressure(void)
{
   return (lPressure);
}

