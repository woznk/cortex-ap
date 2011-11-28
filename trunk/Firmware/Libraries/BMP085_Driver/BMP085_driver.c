//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date: $
// $Author: $
/// \brief
//  Change:
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

/*--------------------------------- Prototypes -------------------------------*/

bmp085_t *p_bmp085 = 0; // pointer to BMP085 device area

///----------------------------------------------------------------------------
///
/// \brief   initialize BMP085
/// \param   *bmp085_t pointer to bmp085 device data structure
/// \return  result of communication routines
/// \remarks the function automatically detects the sensor type and stores
///          this for all future communication and calculation steps
///
///----------------------------------------------------------------------------
int bmp085_init(bmp085_t *bmp085)
{
  char comres=0;
  unsigned char data;
  long dummy;

  /* assign BMP085 ptr */
  p_bmp085 = bmp085;

  /* read Chip Id */
  comres += ReadReg(BMP085_SLAVE_ADDR, BMP085_CHIP_ID__REG, &data);

  p_bmp085->chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
  p_bmp085->number_of_samples = 1;
  p_bmp085->oversampling_setting = 0;

  /* read Version reg */
  comres += ReadReg(BMP085_SLAVE_ADDR, BMP085_VERSION_REG, &data);

  /* get ML Version */
  p_bmp085->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);

  /* get AL Version */
  p_bmp085->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);

  /* readout bmp085 calibration parameter structure */
  bmp085_get_cal_param( );

  return comres;
}


///----------------------------------------------------------------------------
///
/// \brief   read out parameters cal_param from BMP085 memory
/// \param   -
/// \return  result of communication routines
/// \remarks -
///
///----------------------------------------------------------------------------
int bmp085_get_cal_param(void)
{
  int comres;
  unsigned char data[22];

  comres = ReadBuff(BMP085_SLAVE_ADDR, BMP085_PROM_START_ADDR, data, BMP085_PROM_DATA_LEN);

  /* parameters AC1-AC6 */
  p_bmp085->cal_param.ac1 =  (data[0] <<8) | data[1];
  p_bmp085->cal_param.ac2 =  (data[2] <<8) | data[3];
  p_bmp085->cal_param.ac3 =  (data[4] <<8) | data[5];
  p_bmp085->cal_param.ac4 =  (data[6] <<8) | data[7];
  p_bmp085->cal_param.ac5 =  (data[8] <<8) | data[9];
  p_bmp085->cal_param.ac6 = (data[10] <<8) | data[11];

  /* parameters B1,B2 */
  p_bmp085->cal_param.b1 =  (data[12] <<8) | data[13];
  p_bmp085->cal_param.b2 =  (data[14] <<8) | data[15];

  /* parameters MB,MC,MD */
  p_bmp085->cal_param.mb =  (data[16] <<8) | data[17];
  p_bmp085->cal_param.mc =  (data[18] <<8) | data[19];
  p_bmp085->cal_param.md =  (data[20] <<8) | data[21];

  return comres;

}


///----------------------------------------------------------------------------
///
/// \brief   calculate temperature from ut
/// \param   ut, parameter read from device
/// \return  temperature in steps of 0.1 deg celsius
/// \remarks ut was read from the device via I2C and fed into the right calc
///          path for BMP085
///
///----------------------------------------------------------------------------
short bmp085_get_temperature(unsigned long ut)
{
  short temperature;
  long x1,x2,x3,x4,y2,y3,y4;

  x1 = (((long) ut - (long) p_bmp085->cal_param.ac6) * (long) p_bmp085->cal_param.ac5) >> 15;
  x2 = ((long) p_bmp085->cal_param.mc << 11) / (x1 + p_bmp085->cal_param.md);
  p_bmp085->param_b5 = x1 + x2;
  temperature = ((p_bmp085->param_b5 + 8) >> 4);  // temperature in 0.1°C

  return (temperature);
}


///----------------------------------------------------------------------------
///
/// \brief   calculate pressure from up
/// \param   up, parameter read from device
/// \return  pressure in steps of 0.1 Pa
/// \remarks up was read from the device via I2C and fed into the right calc
///          path for BMP085
///
///----------------------------------------------------------------------------
long bmp085_get_pressure(unsigned long up)
{
   long pressure,x1,x2,x3,b3,b6;
   unsigned long b4, b7;

   b6 = p_bmp085->param_b5 - 4000;

   /* calculate B3 */
   x1 = (b6 * b6) >> 12;
   x1 *= p_bmp085->cal_param.b2;
   x1 >>= 11;

   x2 = (p_bmp085->cal_param.ac2 * b6);
   x2 >>= 11;

   x3 = x1 +x2;

   b3 = (((((long)p_bmp085->cal_param.ac1) * 4 + x3) << p_bmp085->oversampling_setting) + 2) >> 2;

   /* calculate B4 */
   x1 = (p_bmp085->cal_param.ac3 * b6) >> 13;
   x2 = (p_bmp085->cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (p_bmp085->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;

   b7 = ((unsigned long)(up - b3) * (50000 >> p_bmp085->oversampling_setting));
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
/// \brief   read out ut for temperature conversion
/// \param   up, parameter read from device
/// \return  uncompensated temperature sensors conversion value
/// \remarks -
///
///----------------------------------------------------------------------------
unsigned short bmp085_get_ut ()
{
  unsigned short ut;
  unsigned char data[2];
  unsigned char ctrl_reg_data;
  int wait_time;
  int comres;

  ctrl_reg_data = BMP085_T_MEASURE;
  wait_time = BMP085_TEMP_CONVERSION_TIME;

  comres = WriteReg(BMP085_SLAVE_ADDR, BMP085_CTRL_MEAS_REG, &ctrl_reg_data);

  p_bmp085->delay_msec (wait_time);
  comres += ReadBuff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 2);

  ut = (data[0] << 8) | data[1];
  return (ut);
}


///----------------------------------------------------------------------------
///
/// \brief   read out up for pressure conversion
/// \param   up, parameter read from device
/// \return  uncompensated pressure value
/// \remarks depending on the oversampling ratio setting up can be 16 to 19 bit
///
///----------------------------------------------------------------------------
unsigned long bmp085_get_up ()
{
  int i;
  unsigned long up=0;
  unsigned char data[3];
  unsigned char ctrl_reg_data;
  int comres=0;

  ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
  comres = p_bmp085->BMP085_BUS_WRITE_FUNC(p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
  p_bmp085->delay_msec ( 2 + (3 << (p_bmp085->oversampling_setting) ) );
  comres += ReadBuff(BMP085_SLAVE_ADDR, BMP085_ADC_OUT_MSB_REG, data, 3);
  up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-p_bmp085->oversampling_setting);
  p_bmp085->number_of_samples = 1;

  return (up);
}


