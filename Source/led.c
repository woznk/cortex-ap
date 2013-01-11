//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief LED driver
///
/// \file
///
// Change: added function LEDStatus to read status of leds
//
//============================================================================*/

#include "stm32f10x.h"
#include "led.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define RED_PIN     GPIO_Pin_9     ///< I/O pin for red LED
#define BLUE_PIN    GPIO_Pin_8     ///< I/O pin for blue LED

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/// pins connected to LEDs
const uint32_t GPIO_PIN[LED_NUM] =
{ RED_PIN, BLUE_PIN };

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   turn on specified led
/// \param   Led = which led
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void LEDOn(Led_TypeDef Led)
{
  GPIOC->BSRR = GPIO_PIN[Led];
}

///----------------------------------------------------------------------------
///
/// \brief   turn off specified led
/// \param   Led = which led
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void LEDOff(Led_TypeDef Led)
{
  GPIOC->BRR = GPIO_PIN[Led];
}

///----------------------------------------------------------------------------
///
/// \brief   toggle status of specified led
/// \param   Led = which led
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void LEDToggle(Led_TypeDef Led)
{
  GPIOC->ODR ^= GPIO_PIN[Led];
}

///----------------------------------------------------------------------------
///
/// \brief   status of specified led
/// \param   Led = which led
/// \return  TRUE if led on, FALSE if led off
/// \remarks -
///
///----------------------------------------------------------------------------
bool LEDStatus(Led_TypeDef Led)
{
  return (bool)((GPIOC->ODR & GPIO_PIN[Led]) != 0);
}
/*****END OF FILE****/
