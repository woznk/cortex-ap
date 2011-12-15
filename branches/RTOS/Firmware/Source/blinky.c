/**
  ******************************************************************************
  * @file TIM/PWM_Output/main.c
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup cortex-ap
  * @{
  */

/** @addtogroup main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_BLINK_RATE    20             /* rate = LED_BLINK_RATE * systick   */
#define LED_NUM            2             /* Number of user LEDs               */
#define USER1              0x0001        /* PA0 : USER1                       */
#define UNBOUNCE_CNT       10            /* unbounce the Push Button          */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t CCR1_Val = 1500;
int16_t CCR2_Val = 1000;
int16_t CCR3_Val = 1500;
int16_t CCR4_Val = 2000;
int16_t CCR1_Delta = 10;
uint32_t ledVal   = 1;
uint32_t ledOn    = 0;
uint32_t ledBlink = 0;
const unsigned long led_mask[] = { 1UL<<8, 1UL<<9 };
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
__INLINE static void LED_Config(void);
__INLINE static void LED_On (uint32_t num);
__INLINE static void LED_Off (uint32_t num);
__INLINE static void BTN_Config(void);
__INLINE static uint32_t BTN_Get(void);
uint32_t BTN_Pressed (void) ;
void LED_Out(uint32_t value);
void SysTick_Handler (void);
void Delay(__IO uint32_t nCount);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  LED_Config();
  BTN_Config();                             
 
  SysTick_Config(SystemCoreClock / 100);       /* Setup SysTick Timer  (10ms) */
  
  while(1) {
    if (BTN_Pressed()) {
      ledVal += 1;
      if (ledVal > 3) ledVal = 1;
    }

    if (ledBlink == 1) {
      ledBlink  = 0;
	  ledOn    ^= 1;
	  if (ledOn == 1)
        LED_Out (ledVal);                      /* switch the LEDs on          */
      else 
        LED_Out (0);                           /* switch the LEDs off         */    
	}
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


/*----------------------------------------------------------------------------
  Configure LED pins
 *----------------------------------------------------------------------------*/
__INLINE static void LED_Config(void) {

  RCC->APB2ENR |=  1 <<  4;                    /* Enable GPIOC clock          */
  GPIOC->CRH   &= 0xFFFFFF00;                  /* Configure the GPIO for LEDs */
  GPIOC->CRH   |= 0x00000033;                  /* Configure the GPIO for LEDs */
}

/*----------------------------------------------------------------------------
  Switch on LEDs
 *----------------------------------------------------------------------------*/
__INLINE static void LED_On (uint32_t num) {

  GPIOC->BSRR = led_mask[num];                 /* Turn On  LED                */
}


/*----------------------------------------------------------------------------
  Switch off LEDs
 *----------------------------------------------------------------------------*/
__INLINE static void LED_Off (uint32_t num) {

  GPIOC->BRR  = led_mask[num];                 /* Turn Off LED                */
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(uint32_t value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}

/*----------------------------------------------------------------------------
  configer Button pins
 *----------------------------------------------------------------------------*/
__INLINE static void BTN_Config(void) {

  RCC->APB2ENR |=  1 <<  2;                    /* Enable GPIOA clock          */
  GPIOA->CRL   &= 0xFFFFFFF0;                  /* Configure the GPIO for BTNs */
  GPIOA->CRL   |= 0x00000008;                  /* Configure the GPIO for BTNs */
}

/*----------------------------------------------------------------------------
  Read Button pins
 *----------------------------------------------------------------------------*/
__INLINE static uint32_t BTN_Get(void) {

 return (GPIOA->IDR & 0x0001);

}


/*----------------------------------------------------------------------------
  USER1Pressed
  check if USER1 is pressed (unbounced).
 *----------------------------------------------------------------------------*/
uint32_t BTN_Pressed (void) {
  static uint32_t USER1KeyCount = 0, USER1KeyPressed = 0;

  if (USER1KeyPressed) {
    if ((BTN_Get() == 0 )) {                   /* Check if USER1 not pressed  */
      if (USER1KeyCount < UNBOUNCE_CNT) USER1KeyCount++;
      else {
        USER1KeyPressed = 0;
        USER1KeyCount = 0;    
      }
    }
  }
  else {
    if (!(BTN_Get() == 0 ))  {                 /* Check if USER1 pressed      */
      if (USER1KeyCount < UNBOUNCE_CNT) USER1KeyCount++;
      else {
        USER1KeyPressed = 1;
        USER1KeyCount = 0;
		return (1);
      }
    }
  }
  return (0);
}
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void) {
  static uint32_t ticks;

  if (ticks++ >= LED_BLINK_RATE) {
    ticks    = 0;
    ledBlink = 1;
  }

}
 

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
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

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
