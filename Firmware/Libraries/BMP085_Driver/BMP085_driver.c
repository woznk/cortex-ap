//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief BMP085 pressure sensor driver
///
//  Change type definitions in unix style
//         renamed variables and function parameters
//
//============================================================================*/

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

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC xBMP85 *pxBMP85 = 0;      // pointer to BMP085 device data structure

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   initialize BMP085
/// \param   -
/// \return  result of communication routines
/// \remarks the function automatically detects the sensor type and stores
///          this for all future communication and calculation steps
///
///----------------------------------------------------------------------------
uint8_t bmp085_init(void)
{
  uint8_t comres = 0;
  uint8_t data;

  /* read Chip Id */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_CHIP_ID__REG, &data);

  pxBMP85->chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
  pxBMP85->number_of_samples = 1;
  pxBMP85->oversampling = 0;

  /* read Version register */
  comres += I2C_MEMS_Read_Reg(BMP085_SLAVE_ADDR, BMP085_VERSION_REG, &data);

  /* get ML Version */
  pxBMP85->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);

  /* get AL Version */
  pxBMP85->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);

  /* extract bmp085 calibration parameter structure */
  bmp085_get_cal_param( );

  return comres;
}


///----------------------------------------------------------------------------
///
/// \brief   extract calibration parameters from BMP085 memory
/// \param   -
/// \return  result of communication routines
/// \remarks -
///
///----------------------------------------------------------------------------
int bmp085_get_cal_param(void)
{
  int comres;
  unsigned char data[22];

  comres = I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, 
                              BMP085_PROM_START_ADDR, 
                              data, 
                              BMP085_PROM_DATA_LEN);

  /* parameters AC1-AC6 */
  pxBMP85->cal_param.ac1 =  (data[0] <<8) | data[1];
  pxBMP85->cal_param.ac2 =  (data[2] <<8) | data[3];
  pxBMP85->cal_param.ac3 =  (data[4] <<8) | data[5];
  pxBMP85->cal_param.ac4 =  (data[6] <<8) | data[7];
  pxBMP85->cal_param.ac5 =  (data[8] <<8) | data[9];
  pxBMP85->cal_param.ac6 = (data[10] <<8) | data[11];

  /* parameters B1,B2 */
  pxBMP85->cal_param.b1 =  (data[12] <<8) | data[13];
  pxBMP85->cal_param.b2 =  (data[14] <<8) | data[15];

  /* parameters MB,MC,MD */
  pxBMP85->cal_param.mb =  (data[16] <<8) | data[17];
  pxBMP85->cal_param.mc =  (data[18] <<8) | data[19];
  pxBMP85->cal_param.md =  (data[20] <<8) | data[21];

  return comres;
}


///----------------------------------------------------------------------------
///
/// \brief   calculate temperature from raw temperature
/// \param   raw_t, parameter read from device
/// \return  temperature in steps of 0.1 deg celsius
/// \remarks raw_t was read from the device via I2C and fed into the right
///          calculation path for BMP085
///
///----------------------------------------------------------------------------
int16_t bmp085_get_temperature(uint32_t raw_t)
{
  int16_t temperature;
  int32_t x1, x2;

  x1 = (((int32_t) raw_t - (int32_t) pxBMP85->cal_param.ac6) * (int32_t) pxBMP85->cal_param.ac5) >> 15;
  x2 = ((int32_t) pxBMP85->cal_param.mc << 11) / (x1 + pxBMP85->cal_param.md);
  pxBMP85->param_b5 = x1 + x2;
  temperature = ((pxBMP85->param_b5 + 8) >> 4);  // temperature in 0.1°C

  return (temperature);
}


///----------------------------------------------------------------------------
///
/// \brief   calculate pressure from raw pressure
/// \param   raw_p, parameter read from device
/// \return  pressure in steps of 0.1 Pa
/// \remarks raw_p was read from the device via I2C and fed into the right calc
///          path for BMP085
///
///----------------------------------------------------------------------------
int32_t bmp085_get_pressure(uint32_t raw_p)
{
   int32_t pressure, x1, x2, x3, b3, b6;
   uint32_t b4, b7;

   b6 = pxBMP85->param_b5 - 4000;

   /* calculate B3 */
   x1 = (b6 * b6) >> 12;
   x1 *= pxBMP85->cal_param.b2;
   x1 >>= 11;

   x2 = (pxBMP85->cal_param.ac2 * b6);
   x2 >>= 11;

   x3 = x1 + x2;

   b3 = (((((int32_t)pxBMP85->cal_param.ac1) * 4 + x3) << pxBMP85->oversampling) + 2) >> 2;

   /* calculate B4 */
   x1 = (pxBMP85->cal_param.ac3 * b6) >> 13;
   x2 = (pxBMP85->cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (pxBMP85->cal_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;

   b7 = ((uint32_t)(raw_p - b3) * (50000 >> pxBMP85->oversampling));
   if (b7 < 0x80000000) {
     pressure = (b7 << 1) / b4;
   } else {
     pressure = (b7 / b4) << 1;
   }

   x1 = pressure >> 8;
   x1 *= x1;
   x1 = (x1 * SMD500_PARAM_MG) >> 16;
   x2 = (pressure * SMD500_PARAM_MH) >> 16;
   pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;	// pressure in Pa

   return (pressure);
}


///----------------------------------------------------------------------------
///
/// \brief   read out raw temperature for temperature conversion
/// \param   -
/// \return  uncompensated temperature sensors conversion value
/// \remarks raw_t parameter read from device
///
///----------------------------------------------------------------------------
uint16_t bmp085_get_raw_t (void)
{
  uint16_t raw_t;
  uint8_t data[2];
//  uint8_t ctrl_reg_data;
  int32_t wait_time, comres;

//  ctrl_reg_data = BMP085_T_MEASURE;
  wait_time = BMP085_TEMP_CONVERSION_TIME;

//  comres = I2C_MEMS_Write_Reg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
  comres = I2C_MEMS_Write_Reg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);

/* REPLACE WITH STAND ALONE FUNCTION */
  pxBMP85->delay_msec ( wait_time );
  comres += I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 2);

  raw_t = (data[0] << 8) | data[1];
  return (raw_t);
}


///----------------------------------------------------------------------------
///
/// \brief   read out up for pressure conversion
/// \param   up, parameter read from device
/// \return  uncompensated pressure value
/// \remarks depending on the oversampling ratio setting up can be 16 to 19 bit
///
///----------------------------------------------------------------------------
uint32_t bmp085_get_raw_p (void)
{
//  int32_t i;
  uint32_t raw_p = 0;
  uint8_t comres = 0;
  uint8_t data[3];
  uint8_t ctrl_reg_data;

  ctrl_reg_data = BMP085_P_MEASURE + (pxBMP85->oversampling << 6);
//  comres = pxBMP85->BMP085_BUS_WRITE_FUNC(pxBMP85->dev_addr, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
  I2C_MEMS_Write_Reg(pxBMP85->dev_addr, BMP085_CTRL_MEAS_REG, ctrl_reg_data);

/* REPLACE WITH STAND ALONE FUNCTION */
  pxBMP85->delay_msec ( 2 + (3 << (pxBMP85->oversampling) ) );
  comres += I2C_MEMS_Read_Buff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 3);
  raw_p = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> (8 - pxBMP85->oversampling);
  pxBMP85->number_of_samples = 1;

  return (raw_p);
}


