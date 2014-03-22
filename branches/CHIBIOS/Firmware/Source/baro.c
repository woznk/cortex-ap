/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date: $
 * $Author: $
 *
 * @brief BMP085 pressure sensor driver
 *
 * Change:
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "baro.h"

/*--------------------------------- Definitions ------------------------------*/

#define BARO_TX_DEPTH    2
#define BARO_RX_DEPTH    3

/* autoincrement bit */
#define AUTO_INCREMENT_BIT      (1 << 7)

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static STRUCT_BMP85 s_bmp85;          /* BMP085 device data structure */
static uint16_t ui_raw_temp;
static int16_t i_temperature;
static int32_t l_raw_press, l_pressure, l_altitude;
static uint8_t baro_tx_data[BARO_TX_DEPTH];
static uint8_t baro_rx_data[BARO_RX_DEPTH];
static i2cflags_t errors = 0;

/*--------------------------------- Prototypes -------------------------------*/

static void compensate_temperature(void);
static void compensate_pressure(void);

/*----------------------------------------------------------------------------
 *
 * @brief   initialize BMP085 barometer
 * @param   -
 * @return  -
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
void Init_Baro(void)
{
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);
  uint8_t calibration[BMP085_PROM_DATA_LEN];

  /* read chip ID register and version register (successive to one another) */
  baro_tx_data[0] = BMP085_CHIP_ID_REG | AUTO_INCREMENT_BIT;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 1, baro_rx_data, 2, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  s_bmp85.chip_id = BMP085_GET_BITSLICE(baro_rx_data[0], BMP085_CHIP_ID);
  s_bmp85.oversampling = 3;
  s_bmp85.ml_version = BMP085_GET_BITSLICE(baro_rx_data[1], BMP085_ML_VERSION);
  s_bmp85.al_version = BMP085_GET_BITSLICE(baro_rx_data[1], BMP085_AL_VERSION);

  /* read calibration parameters */
  baro_tx_data[0] = BMP085_PROM_START_ADDR | AUTO_INCREMENT_BIT;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 1, calibration, BMP085_PROM_DATA_LEN, tmo);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 1, calibration, BMP085_PROM_DATA_LEN, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /* parameters AC1-AC6 */
  s_bmp85.calibration.ac1 =  (calibration[0] << 8) | calibration[1];
  s_bmp85.calibration.ac2 =  (calibration[2] << 8) | calibration[3];
  s_bmp85.calibration.ac3 =  (calibration[4] << 8) | calibration[5];
  s_bmp85.calibration.ac4 =  (calibration[6] << 8) | calibration[7];
  s_bmp85.calibration.ac5 =  (calibration[8] << 8) | calibration[9];
  s_bmp85.calibration.ac6 = (calibration[10] << 8) | calibration[11];

  /* parameters B1,B2 */
  s_bmp85.calibration.b1 =  (calibration[12] << 8) | calibration[13];
  s_bmp85.calibration.b2 =  (calibration[14] << 8) | calibration[15];

  /* parameters MB,MC,MD */
  s_bmp85.calibration.mb =  (calibration[16] << 8) | calibration[17];
  s_bmp85.calibration.mc =  (calibration[18] << 8) | calibration[19];
  s_bmp85.calibration.md =  (calibration[20] << 8) | calibration[21];
}


/*----------------------------------------------------------------------------
 *
 * @brief   BMP084 handler state machine
 * @param   -
 * @return  -
 * @remarks -
 *
 *
 *--------------------------------------------------------------------------*/
void Baro_Handler(void)
{
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /* start temperature conversion */

  baro_tx_data[0] = BMP085_CTRL_MEAS_REG | AUTO_INCREMENT_BIT;
  baro_tx_data[1] = BMP085_T_MEASURE;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 2, baro_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /* wait end of conversion */

  chThdSleepMilliseconds(BMP085_TEMP_CONVERSION_TIME);

  /* read temperature */

  baro_tx_data[0] = BMP085_ADC_OUT_MSB_REG | AUTO_INCREMENT_BIT;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 1, baro_rx_data, 2, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }
  ui_raw_temp = (baro_rx_data[0] << 8) | baro_rx_data[1];
  compensate_temperature();

  /* start pressure conversion */

  baro_tx_data[0] = BMP085_CTRL_MEAS_REG | AUTO_INCREMENT_BIT;
  baro_tx_data[1] = BMP085_P_MEASURE + (s_bmp85.oversampling << 6);
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 2, baro_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /* wait end of conversion */

  chThdSleepMilliseconds(2 + (3 << (s_bmp85.oversampling)));

  /* read uncompensated pressure */

  baro_tx_data[0] = BMP085_ADC_OUT_MSB_REG | AUTO_INCREMENT_BIT;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP085_ADDR, baro_tx_data, 1, baro_rx_data, 3, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  l_raw_press = (((uint32_t) baro_rx_data[0] << 16) |
                 ((uint32_t) baro_rx_data[1] << 8) |
                  (uint32_t) baro_rx_data[2]) >> (8 - s_bmp85.oversampling);

  compensate_pressure();
  l_altitude =(((745 * (11390 - (l_pressure / 10))) / 256 + 46597) * (11390 - (l_pressure / 10))) / 65536 - 966;
}


/*----------------------------------------------------------------------------
 *
 * @brief   compensate temperature
 * @param   -
 * @return  -
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
static void compensate_temperature(void)
{
    int32_t x1, x2;

    x1 = (((int32_t)ui_raw_temp - (int32_t)s_bmp85.calibration.ac6) * (int32_t)s_bmp85.calibration.ac5) >> 15;
    x2 = ((int32_t)s_bmp85.calibration.mc << 11) / (x1 + s_bmp85.calibration.md);
    s_bmp85.param_b5 = x1 + x2;
    i_temperature = ((s_bmp85.param_b5 + 8) >> 4);  /* temperature in 0.1°C */
}


/*----------------------------------------------------------------------------
 *
 * @brief   compensate pressure
 * @param   -
 * @return  -
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
static void compensate_pressure(void)
{
    int32_t x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    /* calculate B6 */
    b6 = s_bmp85.param_b5 - 4000;

    /* calculate B3 */
    x1 = (b6 * b6) >> 12;
    x1 *= s_bmp85.calibration.b2;
    x1 >>= 11;
    x2 = (s_bmp85.calibration.ac2 * b6);
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)s_bmp85.calibration.ac1) * 4 + x3) << s_bmp85.oversampling) + 2) >> 2;

    /* calculate B4 */
    x1 = (s_bmp85.calibration.ac3 * b6) >> 13;
    x2 = (s_bmp85.calibration.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (s_bmp85.calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;

    /* calculate B7 */
    b7 = ((uint32_t)(l_raw_press - b3) * (50000 >> s_bmp85.oversampling));
    if (b7 < 0x80000000) {
      l_pressure = (b7 << 1) / b4;
    } else {
      l_pressure = (b7 / b4) << 1;
    }

    x1 = l_pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (l_pressure * SMD500_PARAM_MH) >> 16;
    l_pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;	/* pressure in Pa */
}

/*----------------------------------------------------------------------------
 *
 * @brief   get compensated temperature
 * @param   -
 * @return  temperature in steps of 0.1 deg celsius
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
int16_t Get_Baro_Temperature(void)
{
  return (i_temperature);
}

/*----------------------------------------------------------------------------
 *
 * @brief   get compensated pressure
 * @param   -
 * @return  pressure in steps of 0.1 Pa
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
int32_t Get_Baro_Pressure(void)
{
   return (l_pressure);
}

/*----------------------------------------------------------------------------
 *
 * @brief   get altitude
 * @param   -
 * @return  altitude in meters
 * @remarks -
 *
 *--------------------------------------------------------------------------*/
int32_t Get_Baro_Altitude(void)
{
   return (l_altitude);
}

