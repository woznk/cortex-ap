/*******************************************************************************
*
* File Name          : L3G4200D.c
* $Revision: $
* $Date: $
* L3G4200D driver file
* Change: basic I2C interface functions moved to i2c_mems_driver,
*         added function L3G4200_Init()
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ADXL345_DRIVER__H
#define __ADXL345_DRIVER__H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/

typedef enum {
  MEMS_SUCCESS = 0x01,
  MEMS_ERROR   = 0x00
} status_t;

typedef enum {
  MEMS_ENABLE  = 0x01,
  MEMS_DISABLE = 0x00
} State_t;


/* Exported constants --------------------------------------------------------*/

// ADXL345 Physical Device Address
#define ADXL345_SLAVE_ADDR  (0x1D << 1)     //

//#define BIT(x) ( 1<<(x) )
#define BIT(x) ( (x) )
#define MEMS_SET        0x01
#define MEMS_RESET      0x00

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
#define ACT_ACDC        BIT(7)
#define ACT_X_ENABLE    BIT(6)
#define ACT_Y_ENABLE    BIT(5)
#define ACT_Z_ENABLE    BIT(4)
#define INACT_ACDC      BIT(3)
#define INACT_X_ENABLE  BIT(2)
#define INACT_Y_ENABLE  BIT(1)
#define INACT_Z_ENABLE  BIT(0)

// Free-fall threshold, R/W, reset value = 00000000
#define THRESH_FF       0x28

// Free-fall time, R/W, reset value = 00000000
#define TIME_FF         0x29

// Axis control for tap/double tap, R/W, reset value = 00000000
#define TAP_AXES        0x2A
#define SUPPRESS        BIT(3)
#define TAP_X ENABLE    BIT(2)
#define TAP_Y ENABLE    BIT(1)
#define TAP_Z ENABLE    BIT(0)

// Source of tap/double tap, R, reset value = 00000000
#define ACT_TAP_STATUS  0x2B
#define ACT_X_SOURCE    BIT(6)
#define ACT_Y_SOURCE    BIT(5)
#define ACT_Z_SOURCE    BIT(4)
#define ASLEEP          BIT(3)
#define TAP_X_SOURCE    BIT(2)
#define TAP_Y_SOURCE    BIT(1)
#define TAP_Z_SOURCE    BIT(0)

// Data rate and power mode control, R/W, reset value = 00001010
#define BW_RATE         0x2C
#define LOW_POWER       BIT(4)
//#define RATE            BIT(3)
//#define RATE            BIT(2)
//#define RATE            BIT(1)
//#define RATE            BIT(0)

// Power-saving features control, R/W, reset value = 00000000
#define POWER_CTL       0x2D
#define LINK            BIT(5)
#define AUTO_SLEEP      BIT(4)
#define MEASURE         BIT(3)
#define SLEEP           BIT(2)
//#define WAKEUP          BIT(1)
//#define WAKEUP          BIT(0)

// Interrupt enable control, R/W, reset value = 00000000
#define INT_ENABLE      0x2E
// Interrupt mapping control, R/W, reset value = 00000000
#define INT_MAP         0x2F
// Interrupt source, R, reset value = 00000010
#define INT_SOURCE      0x30
#define DATA_READY      BIT(7)
#define SINGLE_TAP      BIT(6)
#define DOUBLE_TAP      BIT(5)
#define ACTIVITY        BIT(4)
#define INACTIVITY      BIT(3)
#define FREE_FALL       BIT(2)
#define WATERMARK       BIT(1)
#define OVERRUN         BIT(0)

// Data format control, R/W, reset value = 00000000
#define DATA_FORMAT     0x31
#define SELF_TEST       BIT(7)
#define SPI             BIT(6)
#define INT_INVERT      BIT(5)
#define FULL_RES        BIT(3)
#define Justify         BIT(2)
//#define Range           BIT(1)
//#define Range           BIT(0)

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
//#define FIFO_MODE       BIT(7)
//#define FIFO_MODE       BIT(6)
//#define Trigger         BIT(5)
//#define Samples         BIT(4)
//#define Samples         BIT(3)
//#define Samples         BIT(2)
//#define Samples         BIT(1)
//#define Samples         BIT(0)

// FIFO status, R, reset value = 00000000
#define FIFO_STATUS     0x39
#define FIFO_TRIG       BIT(7)
//#define Entries         BIT(5)
//#define Entries         BIT(4)
//#define Entries         BIT(3)
//#define Entries         BIT(2)
//#define Entries         BIT(1)
//#define Entries         BIT(0)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

// Generic
void ADXL345_Init( void );

#endif /* __L3G4200D_H */




