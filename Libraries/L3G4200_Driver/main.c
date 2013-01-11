/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* $HeadURL $
* $Author: $
* $Revision: 1.5 $
* Changes: SPI_Mems_Init() replaced with I2C_Init()
*
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "led.h"
#include "button.h"
#include "adc_mems.h"
#include "string.h"
#include "l3g4200d_driver.h"
#include "spi_mems.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t response;
uint8_t USBbuffer[64];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  uint8_t status = 0;
  AngRateRaw_t buff;
  int16_t counter = 0;

  InitHardware();

  EKSTM32_LEDOff(LED1);
  EKSTM32_LEDOff(LED2);
  EKSTM32_LEDOff(LED3);

  //wait until the USB is ready
  while(bDeviceState != CONFIGURED);

  //I2C peripheral initialization
  I2C_Init();

  //set the ODR and Bandwith
  SetODR(ODR_100Hz_BW_12_5);
  //enable all axis
  SetAxis(X_ENABLE | Y_ENABLE | Z_ENABLE);
  //set the fullscale
  SetFullScale(FULLSCALE_250);
  //set sensor mode
  SetMode(NORMAL);

  //interrupt pin mode configuration: PUSH PULL
  SetIntPinMode(PUSH_PULL);
  //enable interrutp 1 on INT1 pin and set interrupt active high
  SetInt1Pin(I1_ON_PIN_INT1_ENABLE | INT1_ACTIVE_HIGH);
  //X and Y high threshold interrutps
  SetIntConfiguration(INT1_OR | INT1_ZHIE_ENABLE | INT1_XHIE_ENABLE);
  //interrupt latch disable
  Int1LatchEnable(MEMS_DISABLE);
  //set the threshold only on the Z axis
  SetInt1Threshold(THS_Z, 500);
  //set the duration to 2 odr
  SetInt1Duration(2);

  //FIFO configuration, set the fifo mode
  FIFOModeEnable(FIFO_MODE);

  //set watermark to 5
  SetWaterMark(5);

  //enable watermark interrupt on interrupt2
  //when the fifo contains more than 5 elements, the interrupt raises
  SetInt2Pin(WTM_ON_INT2_ENABLE);

  while (1) {
    //check if there is some data available
    GetSatusReg(&status);
    if (ValBit(status, DATAREADY_BIT)) {

      //get x, y, z angular rate raw data
      GetAngRateRaw(&buff);

      //Led blinking
      if ((counter >= 0) && (counter < 10)) {
        EKSTM32_LEDToggle(LED3);
      }
      counter = (counter + 1) % 100;

      //print on USB
      memset(USBbuffer, '\0', 64);
      sprintf((char*)USBbuffer, "X=%6d Y=%6d Z=%6d \r\n", buff.x, buff.y, buff.z);

      USB_SIL_Write(EP1_IN, USBbuffer, 29 );
      SetEPTxValid(ENDP1);
    }
  }
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
