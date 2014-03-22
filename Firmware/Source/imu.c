/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "imu.h"
#include "config.h"
#include "adxl345.h"
#include "l3g4200d.h"

/* autoincrement bit */
#define AUTO_INCREMENT_BIT      (1 << 7)

/* I2C1 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static i2cflags_t errors = 0;

/* buffers */
static uint8_t imu_rx_data[IMU_RX_DEPTH];
static uint8_t imu_tx_data[IMU_TX_DEPTH];

/* offsets of sensor data */
static int16_t imu_offset[6] = {0,0,0,0,0,0};

/* sign of sensor data */
static const int16_t imu_sign[6] = {
    -1,     /* acceleration X, must be positive forward */
     1,     /* acceleration Y, must be positive rightward */
     1,     /* acceleration Z, must be positive downward */
     1,	    /* roll rate, must be positive when right wing lowers */
    -1,     /* pitch rate, must be positive when tail lowers */
    -1      /* yaw rate, must be positive when turning right */
};

/*----------------------------------------------------------------------------
 *
 * @brief   Initialization of L3G4200D and ADXL345
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
bool Init_IMU ( void ) {

  uint8_t i, j;
  int16_t * p_sensor_data;
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  i2cStart(&I2CD1, &i2cfg1);

  /* tune ports for I2C1*/
  palSetPadMode(IOPORT2, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(IOPORT2, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  chThdSleepMilliseconds(100);  /* Just to be safe. */

  /*------------------------------- L3G4200D -------------------------------*/
  /* set the ODR, bandwith, enable axes, NORMAL MODE */
  imu_tx_data[0] = CTRL_REG1 | AUTO_INCREMENT_BIT;/* first register address */
  imu_tx_data[1] = (XEN | YEN | ZEN | PD);    /* CTRL_REG1: 100 Hz, 12.5 BW, all axes, normal */
  imu_tx_data[2] = 0;                         /* CTRL_REG2: default value */
  imu_tx_data[3] = 0;                         /* CTRL_REG3: default value */
  imu_tx_data[4] = FULLSCALE_2000 << 4;       /* CTRL_REG4: set 2000 deg/s range */
  imu_tx_data[5] = 0;                         /* CTRL_REG5: disable FIFO */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, imu_tx_data, 6, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /*-------------------------------- ADXL345 --------------------------------*/
  /* disable FIFO */
  imu_tx_data[0] = FIFO_CTL;      /* FIFO_CTL register address */
  imu_tx_data[1] = 0;             /* BYPASS_MODE */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 2, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* set full scale range */
  imu_tx_data[0] = DATA_FORMAT;   /* DATA_FORMAT register address */
  imu_tx_data[1] = RANGE_8G;      /* 8 g */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 2, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* set output rate and start measure */
  imu_tx_data[0] = BW_RATE | AUTO_INCREMENT_BIT;  /* BW_RATE register address */
  imu_tx_data[1] = RATE_100HZ;    /* 100 Hz */
  imu_tx_data[2] = MEASURE;       /* POWER_CTL register address, start measure */

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 3, imu_rx_data, 0, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
  }

  /* Compute sensor offsets */
  for (i = 0; i < 16; i++) {
    chThdSleepMilliseconds(32);

  /*-------------------------------- ADXL345 --------------------------------*/

    imu_tx_data[0] = DATAX0 | AUTO_INCREMENT_BIT; /* register address */
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 1, imu_rx_data, 6, tmo);
    i2cReleaseBus(&I2CD1);
    if (status != RDY_OK) {
      errors = i2cGetErrors(&I2CD1);
    }

  /*------------------------------- L3G4200D -------------------------------*/

    imu_tx_data[0] = OUT_X_L | AUTO_INCREMENT_BIT; /* register address */
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, imu_tx_data, 1, &imu_rx_data[6], 6, tmo);
    i2cReleaseBus(&I2CD1);
    if (status != RDY_OK) {
      errors = i2cGetErrors(&I2CD1);
    }

    p_sensor_data = (int16_t *)imu_rx_data;
    for (j = 0; j < 6; j++) {                   /* accumulate */
      imu_offset[j] += *p_sensor_data++;
    }
  }
  for (j = 0; j < 6; j++) {                     /* average */
    imu_offset[j] = imu_offset[j] / 16;
  }

  return 0;
}


/*----------------------------------------------------------------------------
 *
 * @brief   Read acceleration and rotation
 * @return  pointer to 6 elements array containing IMU data
 * @param   -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
int16_t * Request_IMU_Data( void ) {

  uint8_t j;
  int16_t * p_sensor_data;

  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);

  /*-------------------------------- ADXL345 --------------------------------*/

  imu_tx_data[0] = DATAX0 | AUTO_INCREMENT_BIT; /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, ADXL345_ADDR, imu_tx_data, 1, imu_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /*------------------------------- L3G4200D -------------------------------*/

  imu_tx_data[0] = OUT_X_L | AUTO_INCREMENT_BIT; /* register address */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, L3G4200_ADDR, imu_tx_data, 1, &imu_rx_data[6], 6, tmo);
  i2cReleaseBus(&I2CD1);
  if (status != RDY_OK) {
    errors = i2cGetErrors(&I2CD1);
  }

  /* Offset and sign correction */
  p_sensor_data = (int16_t *)imu_rx_data;
  for (j = 0; j < 6; j++) {
    *p_sensor_data -= imu_offset[j];      /* strip offset */
    *p_sensor_data *= imu_sign[j];        /* correct sign */
    if (j == 2) {                         /* z acceleration */
      *p_sensor_data += (int16_t)GRAVITY; /* add gravity */
    }
    p_sensor_data++;
  }
  return (int16_t *)imu_rx_data;
}


