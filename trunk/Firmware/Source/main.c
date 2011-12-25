//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief main program
// Change: LED functions moved to led.c
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f10x.h"

#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"
#include "adxl345_driver.h"
#include "servodriver.h"
#include "diskio.h"

#include "nav.h"
#include "log.h"
#include "led.h"

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
VAR_STATIC uint8_t buff[16];

/*--------------------------------- Prototypes -------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);

void Read_Sensors_Task(void *pvParameters);

///----------------------------------------------------------------------------
///
/// \brief   main
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file     */

  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /* Initialize PWM timers as servo outputs */
//  Servo_Init();

  // I2C peripheral initialization
//  I2C_MEMS_Init();

  // L3G4200 gyro sensor initialization
//  L3G4200_Init();

  // ADXL345 accelerometer sensor initialization
//  ADXL345_Init();

//  while (!Nav_Init());  // Navigation init

//  Log_Init();

  xTaskCreate(Read_Sensors_Task, ( signed portCHAR * ) "Sensors", 16, NULL, 5, NULL);
  xTaskCreate(disk_timerproc, ( signed portCHAR * ) "Disk", 16, NULL, 5, NULL);

  vTaskStartScheduler();

  while (1) {
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
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                         RCC_APB2Periph_USART1, ENABLE);
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

  /* GPIOA Configuration */

  // TIM3 Channel 1, 2 as alternate function push-pull (6, 7)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // USART 1 TX pin as alternate function push pull (9)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // USART 1 RX pin as input floating (10)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOB Configuration */

  // TIM3 Channel 3, 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOC Configuration */

  // LED pins as push pull outputs
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void Dummy_Task(void *pvParameters)
{
	while (1) {
	}
}


///----------------------------------------------------------------------------
///
/// \brief   Read sensors.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Read_Sensors_Task(void *pvParameters)
{
    portTickType Last_Wake_Time;

    Last_Wake_Time = xTaskGetTickCount();

    LEDToggle(GREEN);  // Toggle green LED

    while (1) {

        vTaskDelayUntil(&Last_Wake_Time,100);

//        GetAngRateRaw(buff);                // get x, y, z angular rate raw data
//        GetAccelRaw((uint8_t *)&buff[6]);   // get x, y, z acceleration raw data
//        Log_Send((uint16_t *)buff, 6);      // output data

        LEDToggle(GREEN);  // Toggle green LED
    }
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
