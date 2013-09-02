//============================================================================
//
// fILE NAME
// $Revision: $
// $Date: $
// $Author: $
/// \file
/// \brief  BMP085 lPressure sensor driver
///
//  Change member oversampling of struct STRUCT_BMP85 defined as unsigned
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

/*    CHIP_TYPE CONSTANTS */
#define BOSCH_PRESSURE_BMP085       85

/* BMP085 Physical Device Address */
#define BMP085_SLAVE_ADDR           0xEE

/* BMP085 Identifier */
#define I_AM_BMP085                 0x55

/* SMB380 API error codes*/
#define E_BMP_NULL_PTR              (int8_t)-127
#define E_BMP_COMM_RES              (int8_t)-1
#define E_BMP_OUT_OF_RANGE          (int8_t)-2
#define E_SENSOR_NOT_DETECTED       (int8_t) 0

/* Register definition */
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
#define BMP085_PRESS_CONVERSION_TIME 20

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

/* SMD500 specific constants */
#define SMD500_PARAM_M1     -2218        // calibration parameter
#define SMD500_PARAM_M2      -457        // calibration parameter
#define SMD500_PARAM_M3     -1984        // calibration parameter
#define SMD500_PARAM_M4      8808        // calibration parameter
#define SMD500_PARAM_M5       496        // calibration parameter
#define SMD500_PARAM_M6      1415        // calibration parameter

#define SMD500_PARAM_MB     -4955        // calibration parameter
#define SMD500_PARAM_MC     11611        // calibration parameter
#define SMD500_PARAM_MD    -12166        // calibration parameter
#define SMD500_PARAM_ME    -17268        // calibration parameter
#define SMD500_PARAM_MF     -8970        // calibration parameter

#define SMD500_PARAM_MG      3038        // calibration parameter
#define SMD500_PARAM_MH     -7357        // calibration parameter
#define SMD500_PARAM_MI      3791        // calibration parameter
#define SMD500_PARAM_MJ     64385        // calibration parameter

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
} STRUCT_CALIBRATION;

/* BMP085 image registers data structure */
typedef struct  {
    STRUCT_CALIBRATION calibration;
    uint8_t chip_id, ml_version, al_version;
    uint32_t param_b5;
    uint16_t oversampling;
} STRUCT_BMP85;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

// Generic functions
uint8_t BMP085_Init(void);
void BMP085_Handler(void);
int16_t BMP085_Get_Temperature(void);
int32_t BMP085_Get_Pressure(void);
int32_t BMP085_Get_Altitude(void);

#endif /* __BMP085_DRIVER__H */

