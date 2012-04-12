//============================================================================
//
// fILE NAME
// $Revision: $
// $Date: $
// $Author: $
/// \file
/// \brief  BMP085 pressure sensor driver
///
//  Change type definitions in unix style
//         renamed variables and function parameters
//
//============================================================================

/*-------------------------- Prevent recursive inclusion ---------------------*/

#ifndef __BMP085_DRIVER__H
#define __BMP085_DRIVER__H

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

// Define the calling convention of YOUR bus communication routine.
// note This includes types of parameters. This example shows the configuration for an SPI bus link.

// defines the return parameter type of the BMP085_WR_FUNCTION
#define BMP085_BUS_WR_RETURN_TYPE char

// defines the calling parameter types of the BMP085_WR_FUNCTION
#define BMP085_BUS_WR_PARAM_TYPES unsigned char,unsigned char,unsigned char *,unsigned char

// links the order of parameters defined in BMP085_BUS_WR_PARAM_TYPE to function calls used inside the API
#define BMP085_BUS_WR_PARAM_ORDER device_addr, register_addr, register_data, write_length

// never change this line
#define BMP085_BUS_WRITE_FUNC(device_addr, register_addr, register_data, write_length)\
           bus_write( device_addr, register_addr, register_data, write_length )

// defines the return parameter type of the BMP085_WR_FUNCTION
#define BMP085_BUS_RD_RETURN_TYPE char

// defines the calling parameter types of the BMP085_WR_FUNCTION
#define BMP085_BUS_RD_PARAM_TYPES unsigned char, unsigned char, unsigned char *, unsigned char

// links the order of parameters defined in BMP085_BUS_WR_PARAM_TYPE to function calls used inside the API
#define BMP085_BUS_RD_PARAM_ORDER device_addr, register_addr, register_data, read_length

// never change this line */
#define BMP085_BUS_READ_FUNC(device_addr, register_addr, register_data, read_length)\
           bus_read( device_addr, register_addr, register_data, read_length )

/*    CHIP_TYPE CONSTANTS */
#define BOSCH_PRESSURE_BMP085       85

// BMP085 Physical Device Address
#define BMP085_SLAVE_ADDR           (0xEE << 1)

#define I_AM_BMP085                 0x55

/*    SMB380 API error codes*/
#define E_BMP_NULL_PTR              (int8_t)-127
#define E_BMP_COMM_RES              (int8_t)-1
#define E_BMP_OUT_OF_RANGE          (int8_t)-2
#define E_SENSOR_NOT_DETECTED       (int8_t) 0

// Register definition

#define BMP085_PROM_START_ADDR      0xAA
#define BMP085_PROM_DATA_LEN        22

#define BMP085_CHIP_ID_REG          0xD0
#define BMP085_VERSION_REG          0xD1

#define BMP085_CTRL_MEAS_REG        0xF4
#define BMP085_ADC_OUT_MSB_REG      0xF6
#define BMP085_ADC_OUT_LSB_REG      0xF7

#define BMP085_SOFT_RESET_REG       0xE0

#define BMP085_T_MEASURE            0x2E        // temperature measurent
#define BMP085_P_MEASURE            0x34        // pressure measurement

#define BMP085_TEMP_CONVERSION_TIME 5           // TO be spec'd by GL or SB

/* register write and read delays */
#define BMP085_MDELAY_DATA_TYPE    uint16_t
#define BMP085_MDELAY_RETURN_TYPE  void

/* bit slice positions in registers */
#define BMP085_CHIP_ID__POS         0
#define BMP085_CHIP_ID__MSK         0xFF
#define BMP085_CHIP_ID__LEN         8
#define BMP085_CHIP_ID__REG         BMP085_CHIP_ID_REG

#define BMP085_ML_VERSION__POS      0
#define BMP085_ML_VERSION__LEN      4
#define BMP085_ML_VERSION__MSK      0x0F
#define BMP085_ML_VERSION__REG      BMP085_VERSION_REG

#define BMP085_AL_VERSION__POS      4
#define BMP085_AL_VERSION__LEN      4
#define BMP085_AL_VERSION__MSK      0xF0
#define BMP085_AL_VERSION__REG      BMP085_VERSION_REG


/*----------------------------------- Macros ---------------------------------*/

/* DATA REGISTERS */

/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */
#define BMP085_GET_BITSLICE(regvar, bitname)\
            (regvar & bitname##__MSK) >> bitname##__POS

#define BMP085_SET_BITSLICE(regvar, bitname, val)\
          (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/* this structure holds all device specific calibration parameters */
typedef struct {
   int16_t ac1;
   int16_t ac2;
   int16_t ac3;
   uint16_t ac4;
   uint16_t ac5;
   uint16_t ac6;
   int16_t b1;
   int16_t b2;
   int16_t mb;
   int16_t mc;
   int16_t md;
} bmp085_calibration_param_t;


/* BMP085 image registers data structure */
typedef struct  {
    bmp085_calibration_param_t cal_param;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
    uint32_t param_b5;
    uint16_t number_of_samples;
    int16_t oversampling;
    int16_t smd500_t_resolution, smd500_masterclock;
    BMP085_MDELAY_RETURN_TYPE (*delay_msec)( BMP085_MDELAY_DATA_TYPE );
} xBMP85;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

// Generic functions
uint8_t bmp085_init(void);
int16_t bmp085_get_temperature(uint32_t raw_t);
int32_t bmp085_get_pressure(uint32_t raw_p);
uint16_t bmp085_get_raw_t(void);
uint32_t bmp085_get_raw_p(void);

// API internal helper functions
int32_t bmp085_get_cal_param(void);
//int32_t smd500_get_cal_param(void);

#endif /* __BMP085_DRIVER__H */

