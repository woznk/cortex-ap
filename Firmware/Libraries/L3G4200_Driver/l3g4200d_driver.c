///----------------------------------------------------------------------------
///
/// File Name: L3G4200D_Driver.c
/// $Revision:$
/// $Date:$
/// \brief
/// L3G4200D driver file
///
/// Changes: (Lint) unsigned char replaced with uint8_t, casted several 
///          variables to uint8_t, non void function calls casted to (void)
///
///----------------------------------------------------------------------------

#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : SetODR
* Description    : Sets L3G4200D Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetODR(ODR_t ov) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, &value) )
    return MEMS_ERROR;

  value &= 0x0f;
  value |= ((uint8_t)ov << 4);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetMode
* Description    : Sets L3G4200D Operating Mode
* Input          : Modality (NORMAL, SLEEP, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetMode(Mode_t md) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, &value) )
    return MEMS_ERROR;

  switch(md) {

  case POWER_DOWN:
    value &= 0xF7;
    value |= (MEMS_RESET<<PD);
    break;

  case NORMAL:
    value &= 0xF7;
    value |= (MEMS_SET<<PD);
    break;

  case SLEEP:
    value &= 0xF0;
    value |= ((MEMS_SET<<PD) | (MEMS_RESET<<ZEN) | (MEMS_RESET<<YEN) | (MEMS_RESET<<XEN));
    break;

  default:
    return MEMS_ERROR;
  }

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetAxis
* Description    : Enable/Disable L3G4200D Axis
* Input          : X_ENABLE/X_DISABLE | Y_ENABLE/Y_DISABLE | Z_ENABLE/Z_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetAxis(Axis_t axis) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, &value) )
    return MEMS_ERROR;

  value &= 0xf8;
  value |= axis;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG1, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetFullScale
* Description    : Sets the L3G4200D FullScale
* Input          : FS_250/FS_500/FS_2000
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetFullScale(Fullscale_t fs) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= 0xCF;
  value |= ((uint8_t)fs << FS);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetBDU(State_t bdu) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= 0x7F;
  value |= ((uint8_t)bdu << BDU);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetBLE(Endianess_t ble) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= 0xBF;
  value |= ((uint8_t)ble << BLE);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSelfTest
* Description    : Set Self Test Modality
* Input          : ST_NORMAL_MODE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSelfTest(SelfTest_t st) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= 0xF9;
  value |= ((uint8_t)st << SELF_TEST);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : HPFEnable
* Description    : Enable/Disable High Pass Filter
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t HPFEnable(State_t hpf) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, &value) )
    return MEMS_ERROR;

  value &= 0xEF;
  value |= ((uint8_t)hpf << HPEN);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : HPM_NORMAL_MODE_RES/HPM_REF_SIGNAL/HPM_NORMAL_MODE/HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetHPFMode(HPFMode_t hpf) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG2, &value) )
    return MEMS_ERROR;

  value &= 0xCF;
  value |= ((uint8_t)hpf << HPM);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG2, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF_0,HPFCF_1,HPFCF_2... See Table 27 of the datasheet
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetHPFCutOFF(HPFCutOffFreq_t hpf) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG2, &value) )
    return MEMS_ERROR;

  value &= 0xF0;
  value |= ((uint8_t)hpf << HPFC0);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG2, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetIntPinMode
* Description    : Set Interrupt Pin Modality (push pull or open drain)
* Input          : PUSH_PULL/OPEN_DRAIN
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetIntPinMode(IntPinMode_t pm) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, &value) )
    return MEMS_ERROR;

  value &= 0xEF;
  value |= ((uint8_t)pm << PP_OD);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          : I1_ON_PIN_INT1_ENABLE | I1_BOOT_ON_INT1 | INT1_ACTIVE_HIGH
* example        : SetInt1Pin(I1_ON_PIN_INT1_ENABLE | I1_BOOT_ON_INT1_ENABLE |
                   INT1_ACTIVE_LOW) to enable Interrupt 1 or Bootsatus on
				   interrupt 1 pin
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetInt1Pin(Int1PinConf_t pinConf) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, &value) )
    return MEMS_ERROR;

  value &= 0x1F;
  value |= pinConf;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : I2_DRDY_ON_INT2_ENABLE/DISABLE |
                   WTM_ON_INT2_ENABLE/DISABLE |
                   OVERRUN_ON_INT2_ENABLE/DISABLE |
                   EMPTY_ON_INT2_ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetInt2Pin(Int2PinConf_t pinConf) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, &value) )
    return MEMS_ERROR;

  value &= 0xF0;
  value |= pinConf;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG3, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t Int1LatchEnable(State_t latch) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,INT1_CFG, &value) )
    return MEMS_ERROR;

  value &= 0xBF;
  value |= ((uint8_t)latch << LIR);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_CFG, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t ResetInt1Latch(void) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,INT1_SRC, &value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetIntConfiguration
* Description    : Interrupt 1 Configuration
* Input          : AND/OR, INT1_LIR ZHIE_ENABLE/DISABLE | INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetIntConfiguration(Int1Conf_t ic) {
  uint8_t value;

  value = ic;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_CFG, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetInt1Threshold(IntThsAxis axis, uint16_t ths) {
  uint8_t value;

  switch (axis) {

    case THS_X:
      //write the threshold LSB
      value = (uint8_t)(ths & 0x00ff);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_XL, value) )
        return MEMS_ERROR;

      //write the threshold LSB
      value = (uint8_t)(ths >> 8);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_XH, value) )
        return MEMS_ERROR;
      break;

    case THS_Y:
      //write the threshold LSB
      value = (uint8_t)(ths & 0x00ff);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_YL, value) )
        return MEMS_ERROR;

      //write the threshold LSB
      value = (uint8_t)(ths >> 8);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_YH, value) )
        return MEMS_ERROR;
      break;

    case THS_Z:
      //write the threshold LSB
      value = (uint8_t)(ths & 0x00ff);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_ZL, value) )
        return MEMS_ERROR;

      //write the threshold LSB
      value = (uint8_t)(ths >> 8);
      if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_THS_ZH, value) )
        return MEMS_ERROR;
      break;
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetInt1Duration(Int1Conf_t id) {

  if (id > 127)
    return MEMS_ERROR;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,INT1_DURATION, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          :
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t FIFOModeEnable(FifoMode_t fm) {
  uint8_t value;

  if (fm == FIFO_DISABLE) {

    if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, &value) )
      return MEMS_ERROR;

    value &= 0xBF;

    if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, value) )
      return MEMS_ERROR;

  } else {

    if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, &value) )
      return MEMS_ERROR;

    value &= 0xBF;
    value |= MEMS_SET<<FIFO_EN;

    if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, value) )
      return MEMS_ERROR;


    if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= ((uint8_t)fm << FM0);

    if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetWaterMark(uint8_t wtm) {
  uint8_t value;

  if (wtm > 31)
    return MEMS_ERROR;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;

  value &= 0xE0;
  value |= wtm;

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,FIFO_CTRL_REG, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetSatusReg
* Description    : Read the status register
* Input          : None
* Output         : status register buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetStatusReg(uint8_t* buff) {

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR, STATUS_REG, buff) )
      return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetAngRateRaw
* Description    : Read the Angular Rate Registers
* Input          : None
* Output         : Angular Rate Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
bool GetAngRateRaw(uint8_t* data) {
/*
  uint8_t status;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR, STATUS_REG, &status)) {
     return FALSE;
  }

  if (!ValBit(status, DATAREADY_BIT)) {
     return FALSE;
  }
*/
  if (!I2C_MEMS_Read_Buff(L3G4200_SLAVE_ADDR, (OUT_X_L | AUTO_INCR), data, 6)) {
     return FALSE;
  }

  return TRUE;
}


/*******************************************************************************
* Function Name  : GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetInt1Src(uint8_t* buff) {

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,INT1_SRC, buff) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t GetFifoSourceReg(uint8_t* buff) {

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,FIFO_SRC_REG, buff) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetOutputDataAndFifoFilters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to output and fifo registers
*                : See Table 8 of AN3393 for more details
* Input          : NONE, HPH, LPF2, HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetOutputDataAndFifoFilters(HPF_LPF2_Enable hpf){
  uint8_t value;

  //HPF
  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, &value) )
    return MEMS_ERROR;

  switch(hpf) {

  case NONE:
    value &= 0xfc;
    value |= 0x00; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 0
    break;

  case HPF:
    value &= 0xfc;
    value |= 0x01; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 1
    break;

  case LPF2:
    value &= 0xed;
    value |= 0x02; //hpen = 0, Out_sel_1 = 1, Out_sel_0 = x
    break;

  case HPFLPF2:
    value &= 0xed;
    value |= 0x12; //hpen = 1, Out_sel_1 = 1, Out_sel_0 = x
    break;
  }

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, value) )
    return MEMS_ERROR;


  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetInt1Filters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to Int1 circuitry
*                : See Table 9 of AN3393 for more details
* Input          : NONE, HPH, LPF2, HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetInt1Filters(HPF_LPF2_Enable hpf){
  uint8_t value;

  //HPF
  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, &value) )
    return MEMS_ERROR;

  switch(hpf) {

  case NONE:
    value &= 0xf3;
    value |= 0x00<<INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 0
    break;

  case HPF:
    value &= 0xf3;
    value |= 0x01<<INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 1
    break;

  case LPF2:
    value &= 0xe7;
    value |= 0x02<<INT1_SEL0; //hpen = 0, Int1_sel_1 = 1, Int1_sel_0 = x
    break;

  case HPFLPF2:
    value &= 0xe7;
    value |= 0x01<<HPEN;
    value |= 0x02<<INT1_SEL0; //hpen = 1, Int1_sel_1 = 1, Int1_sel_0 = x
    break;
  }

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG5, value) )
    return MEMS_ERROR;


  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : SPI_3_WIRE, SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetSPIInterface(SPIMode_t spi) {
  uint8_t value;

  if (!I2C_MEMS_Read_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= 0xFE;
  value |= ((uint8_t)spi << SIM);

  if (!I2C_MEMS_Write_Reg(L3G4200_SLAVE_ADDR,CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3G4200_Init
* Description    : Initialize L3G4200 sensor
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void L3G4200_Init( void )
{
  //set the ODR and Bandwith
  (void)SetODR(ODR_100Hz_BW_12_5);
  //enable all axis
  (void)SetAxis((Axis_t)((uint8_t)X_ENABLE | (uint8_t)Y_ENABLE | (uint8_t)Z_ENABLE));
  //set the fullscale
  (void)SetFullScale(FULLSCALE_2000);
  //set sensor mode
  (void)SetMode(NORMAL);
  //bypass fifo
  (void)FIFOModeEnable(FIFO_BYPASS_MODE);
}
