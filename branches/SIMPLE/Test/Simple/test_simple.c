//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief main program
///
/// \file
///
/// \todo
/// 1) Move task definitions in the main.c file, export relevant functions
/// from specific modules and call them from inside task, this should improve
/// testability.
///
/// \todo
/// 2) Use only one data structure for SD file read/write, add a semaphore
/// to manage multiple accesses, this will reduce RAM usage by 512 bytes.
///
// Change: replaced GPIO and RCC configuration functions with HW_Init()
//
//============================================================================*/
/*
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
*/
#include "stm32f10x.h"
/*
#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"
#include "adxl345_driver.h"
#include "servodriver.h"
#include "ppmdriver.h"
#include "usart1driver.h"
#include "diskio.h"

#include "config.h"
// uncomment telemetry type that applies
//#include "telemetry.h"
#include "mav_telemetry.h"
//#include "multiwii.h"
#include "attitude.h"
#include "log.h"
#include "nav.h"
*/
#include "led.h"

/** @addtogroup cortex_ap
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

/*--------------------------------- Prototypes -------------------------------*/

void HW_Init(void);

/*--------------------------------- Functions --------------------------------*/


///----------------------------------------------------------------------------
///
/// \brief   main
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
int32_t main(void) {

  /* At this stage the microcontroller clock setting is already configured,
     this is done through SystemInit() function which is called from startup
     file (startup_stm32f10x_xx.s) before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f10x.c file */

  SCB->AIRCR = 0x05FA0300; // 4 bits for pre-emption priority, 0 bits for subpriority
              // |  ||  |
              // |  |+--+- NVIC_PriorityGroup_4
              // |  |
              // +--+----- AIRCR_VECTKEY_MASK

  HW_Init();              // Initialize hardware
  USART1_Init();          // Initialize USART1 for telemetry

  while (1) {
      LEDOn(BLUE);
      LEDOff(BLUE);
      LEDToggle(BLUE);
  }
}

///----------------------------------------------------------------------------
///
/// \brief   Configure the hardware
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void HW_Init(void)
{
#define GPIOA_CRL_MASK  0x00FF000F
#define GPIOA_CRL_BITS  0xAA004A80
                     //   ||  |||
                     //   ||  ||+- Pin  1: TIM2 CH 2 input
                     //   ||  |+-- Pin  2: USART 2 TX alternate push pull 2 MHz
                     //   ||  +--- Pin  3: USART 2 RX input floating
                     //   |+------ Pin  6: TIM3 CH 1 alternate push pull 2 MHz
                     //   +------- Pin  7: TIM3 CH 2 alternate push pull 2 MHz

#define GPIOA_CRH_MASK  0xFFFFF00F
#define GPIOA_CRH_BITS  0x000004A0
                    //         ||
                    //         |+- Pin  9: USART 1 TX alternate push pull 2 MHz
                    //         +-- Pin 10: USART 1 RX input floating

#define GPIOA_BSRR_BITS  0x00020000
                     //       |
                     //       +--- Pin  1: TIM2 CH 2 pull down

#define GPIOB_CRL_MASK  0xFFFFFF00
#define GPIOB_CRL_BITS  0x000000AA
                     //         ||
                     //         |+- Pin  0: TIM3 CH 3 alternate push pull 2 MHz
                     //         +-- Pin  1: TIM3 CH 4 alternate push pull 2 MHz

#define GPIOC_CRH_MASK  0xFFFFFF00
#define GPIOC_CRH_BITS  0x00000022
                     //         ||
                     //         |+- Pin  8: LED output push pull 2 MHz
                     //         +-- Pin  9: LED output push pull 2 MHz

  uint32_t tmpreg;

  tmpreg = RCC->CFGR;
  tmpreg &= 0xFFFFC00F; // Clear PPRE2[2:0] PPRE1[2:0] HPRE[3:0] bits
  tmpreg |= 0x00000400; // Set HPRE[3:0] (HCLK=SYSCLK), PPRE2[2:0] (PCLK1=HCLK), PPRE1[2:0] (PLCK2=HCLK/2)
  RCC->CFGR = tmpreg;   // Store the new value

  tmpreg = RCC->APB1ENR;
  tmpreg |= 0x00020003;
            //   |   |
            //   |   +- Timer 2 used for PPM signal capture (BIT 1)
            //   |   +- Timer 3 used for servo signal PWM (BIT 2)
            //   +----- Usart 2 used for GPS communication (BIT 17)
  RCC->APB1ENR = tmpreg;

  tmpreg = RCC->APB2ENR;
  tmpreg |= 0x0000401D;
            //    | ||
            //    | |+- AFIO used for ... (BIT 0)
            //    | |+- GPIOA used by TIM2, TIM3, USART1, USART2 (BIT 2)
            //    | |+- GPIOB used for TIM3 (BIT 3)
            //    | +-- GPIOC used for LEDs (BIT 4)
            //    +---- USART 1 used for telemetry (BIT 14)
  RCC->APB2ENR = tmpreg;

  tmpreg = GPIOA->CRL;             // Port A pins 0 - 7
  tmpreg &= GPIOA_CRL_MASK;
  tmpreg |= GPIOA_CRL_BITS;
  GPIOA->CRL = tmpreg;

  GPIOA->BSRR = GPIOA_BSRR_BITS;    // TIM2 CH 2 pull down

  tmpreg = GPIOA->CRH;             // Port A pins 8 - 15
  tmpreg &= GPIOA_CRH_MASK;
  tmpreg |= GPIOA_CRH_BITS;
  GPIOA->CRH = tmpreg;

  tmpreg = GPIOB->CRL;             // Port B pins 0 - 1
  tmpreg &= GPIOB_CRL_MASK;
  tmpreg |= GPIOB_CRL_BITS;
  GPIOB->CRL = tmpreg;

  tmpreg = GPIOC->CRH;             // Port C pins 8 - 15
  tmpreg &= GPIOC_CRH_MASK;
  tmpreg |= GPIOC_CRH_BITS;
  GPIOC->CRH = tmpreg;
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
