//============================================================================
//
// fILE NAME
// $Revision: $
// $Date: $
// $Author: $
/// \file
/// \brief  ADXL345 driver
//  Change: Added definitions for output rate and full scale range
//
//============================================================================

/*-------------------------- Prevent recursive inclusion ---------------------*/

#ifndef __ADXL345_DRIVER__H
#define __ADXL345_DRIVER__H

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

// ADXL345 Physical Device Address
#define ADXL345_SLAVE_ADDR  (0x1D << 1)     //

#define I_AM_ADXL345    0xE5

// Register definition

// Device ID, R, reset value = 11100101.
#define DEVID           0x00

// 0x01 - 0x1C   Reserved, do not access.

// Tap threshold, R/W, reset value = 00000000
#define THRESH_TAP      0x1D

// X-axis offset, R/W, reset value = 00000000
#define OFSX            0x1E

// Y-axis offset, R/W, reset value = 00000000
#define OFSY            0x1F

// Z-axis offset, R/W, reset value = 00000000
#define OFSZ            0x20

// Tap duration, R/W, reset value = 00000000
#define DUR             0x21

// Tap latency, R/W, reset value = 00000000
#define LATENT          0x22

// Tap window, R/W, reset value = 00000000
#define WINDOW          0x23

// Activity threshold, R/W, reset value = 00000000
#define THRESH_ACT      0x24

// Inactivity threshold, R/W, reset value = 00000000
#define THRESH_INACT    0x25

// Inactivity time, R/W, reset value = 00000000
#define TIME_INACT      0x26

// Axis enable control for activity and inactivity detection, R/W, reset value = 00000000
#define ACT_INACT_CTL   0x27
#define ACT_ACDC        0x80
#define ACT_X_ENABLE    0x40
#define ACT_Y_ENABLE    0x20
#define ACT_Z_ENABLE    0x10
#define INACT_ACDC      0x08
#define INACT_X_ENABLE  0x04
#define INACT_Y_ENABLE  0x02
#define INACT_Z_ENABLE  0x01

// Free-fall threshold, R/W, reset value = 00000000
#define THRESH_FF       0x28

// Free-fall time, R/W, reset value = 00000000
#define TIME_FF         0x29

// Axis control for tap/double tap, R/W, reset value = 00000000
#define TAP_AXES        0x2A
#define SUPPRESS        0x08
#define TAP_X ENABLE    0x04
#define TAP_Y ENABLE    0x02
#define TAP_Z ENABLE    0x01

// Source of tap/double tap, R, reset value = 00000000
#define ACT_TAP_STATUS  0x2B
#define ACT_X_SOURCE    0x40
#define ACT_Y_SOURCE    0x20
#define ACT_Z_SOURCE    0x10
#define ASLEEP          0x08
#define TAP_X_SOURCE    0x04
#define TAP_Y_SOURCE    0x02
#define TAP_Z_SOURCE    0x01

// Data rate and power mode control, R/W, reset value = 00001010
#define BW_RATE         0x2C
#define LOW_POWER       0x10
#define RATE_6HZ        0x06
#define RATE_12HZ       0x07
#define RATE_25HZ       0x08
#define RATE_50HZ       0x09
#define RATE_100HZ      0x0A
#define RATE_200HZ      0x0B
#define RATE_400HZ      0x0C
#define RATE_800HZ      0x0D
#define RATE_1600HZ     0x0E
#define RATE_3200HZ     0x0F

// Power-saving features control, R/W, reset value = 00000000
#define POWER_CTL       0x2D
#define LINK            0x20
#define AUTO_SLEEP      0x10
#define MEASURE         0x08
#define SLEEP           0x04
//#define WAKEUP        0x02
//#define WAKEUP        0x01

// Interrupt enable control, R/W, reset value = 00000000
#define INT_ENABLE      0x2E
// Interrupt mapping control, R/W, reset value = 00000000
#define INT_MAP         0x2F
// Interrupt source, R, reset value = 00000010
#define INT_SOURCE      0x30
#define DATA_READY      0x80
#define SINGLE_TAP      0x40
#define DOUBLE_TAP      0x20
#define ACTIVITY        0x10
#define INACTIVITY      0x08
#define FREE_FALL       0x04
#define WATERMARK       0x02
#define OVERRUN         0x01

// Data format control, R/W, reset value = 00000000
#define DATA_FORMAT     0x31
#define ADXL_SELF_TEST  0x80
#define ADXL_SPI        0x40
#define ADXL_INT_INVERT 0x20
#define ADXL_FULL_RES   0x08
#define ADXL_JUSTIFY    0x04
#define RANGE_2G        0x00
#define RANGE_4G        0x01
#define RANGE_8G        0x02
#define RANGE_16G       0x03

// X-Axis Data 0, R, reset value = 00000000
#define DATAX0          0x32

// X-Axis Data 1, R, reset value = 00000000
#define DATAX1          0x33

// Y-Axis Data 0, R, reset value = 00000000
#define DATAY0          0x34

// Y-Axis Data 1, R, reset value = 00000000
#define DATAY1          0x35

// Z-Axis Data 0, R, reset value = 00000000
#define DATAZ0          0x36

// Z-Axis Data 1, R, reset value = 00000000
#define DATAZ1          0x37

// FIFO control, R/W, reset value = 00000000
#define FIFO_CTL        0x38
#define BYPASS_MODE     0x00
#define FIFO_MODE       0x40
#define STREAM_MODE     0x80
#define TRIGGER_MODE    0xC0
#define TRIGGER         0x20
//#define Samples       0x10
//#define Samples       0x08
//#define Samples       0x04
//#define Samples       0x02
//#define Samples       0x01

// FIFO status, R, reset value = 00000000
#define FIFO_STATUS     0x39
#define FIFO_TRIG       0x80
//#define Entries       0x20
//#define Entries       0x10
//#define Entries       0x08
//#define Entries       0x04
//#define Entries       0x02
//#define Entries       0x01

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef struct{
  short int x;
  short int y;
  short int z;
} AccelRaw_t;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

// Generic
bool ADXL345_Init( void );
bool GetAccelRaw(uint8_t* data);

#endif /* __ADXL345_DRIVER__H */




