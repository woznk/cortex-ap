//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  main program
// Change: SPI_MEMS_Init() replaced with I2C_MEMS_Init()
//
//============================================================================*/

#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "servodriver.h"
#include "l3g4200d_driver.h"
#include "tick.h"
#include "nav.h"
#include "log.h"

/** @addtogroup cortex-ap
  * @{
  */

/** @addtogroup main
  * @{
  */

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

VAR_STATIC int16_t Servo_Position = 1500;
VAR_STATIC int16_t Servo_Delta = 10;

/*--------------------------------- Prototypes -------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(__IO uint32_t nCount);


///----------------------------------------------------------------------------
///
/// \brief   main
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
int main(void)
{
  uint8_t status = 0;
  AngRateRaw_t buff;

  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file     */

  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /* Initialize Leds LD3 and LD4 of STM32VLDISCOVERY board */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);

  /* Setup SysTick Timer  (10ms) */
  SysTick_Config(SystemCoreClock / 100);

  /* Initialize PWM timers as servo outputs */
  Servo_Init();

  //SPI peripheral initialization
  I2C_MEMS_Init();

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
  //set the fifo mode
  FIFOModeEnable(FIFO_MODE);
  //set watermark to 5
  SetWaterMark(5);
  //enable watermark interrupt on interrupt2 
  //when the fifo contains more than 5 elements, the interrupt raises
  SetInt2Pin(WTM_ON_INT2_ENABLE);
/*
  while (!Nav_Init());  // Navigation init
  Log_Init();
*/
  while (1) {
    if ((g_ulFlags & FLAG_CLOCK_TICK_10) != 0) {
        g_ulFlags &= !FLAG_CLOCK_TICK_10;

        STM32vldiscovery_LEDOff(LED3);      // Turn off LD3
        STM32vldiscovery_LEDOff(LED4);      // Turn off LD4
        if (Servo_Delta == 10) {
            STM32vldiscovery_LEDOn(LED3);   // Turn on LD3
        } else {
            STM32vldiscovery_LEDOn(LED4);   // Turn on LD4
        }
        if (Servo_Position > 1999) {
            Servo_Delta = -10;
        } else if (Servo_Position < 999) {
            Servo_Delta = 10;
        }
        Servo_Position += Servo_Delta;
        Servo_Set(SERVO_RUDDER, Servo_Position);
    }

    //check if there is some data available
    GetStatusReg(&status);
    if (ValBit(status, DATAREADY_BIT)) {      
      //get x, y, z angular rate raw data
      GetAngRateRaw(&buff);
    }
  }
}

///----------------------------------------------------------------------------
///
/// \brief   Configure the different system clocks.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void RCC_Configuration(void)
{
  /* HCLK = SYSCLK */
  RCC_HCLKConfig(RCC_SYSCLK_Div1);

  /* PCLK1 = HCLK/2 */
  RCC_PCLK1Config(RCC_HCLK_Div2);

  /* PCLK2 = HCLK */
  RCC_PCLK2Config(RCC_HCLK_Div1);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

///----------------------------------------------------------------------------
///
/// \brief   Configure pins.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM3 Channel 1, 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOC Configuration:TIM3 Channel 3, 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT
///----------------------------------------------------------------------------
///
/// \brief   Reports the name of the source file and the source line number
///          where the assert_param error has occurred.
/// \param   file: pointer to the source file name
/// \param   line: assert_param error line source number
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
