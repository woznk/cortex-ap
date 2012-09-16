//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief LED driver header file
///
/// \file
///
//  Change GREEN LED changed into RED
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

#define GREEN_PIN    GPIO_Pin_9     ///< I/O pin for green LED
#define BLUE_PIN     GPIO_Pin_8     ///< I/O pin for blue LED

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/// LED type definition
typedef enum {
  RED     = 0,
  BLUE    = 1,
  LED_NUM
} Led_TypeDef;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void LEDOn(Led_TypeDef Led);
void LEDOff(Led_TypeDef Led);
void LEDToggle(Led_TypeDef Led);
