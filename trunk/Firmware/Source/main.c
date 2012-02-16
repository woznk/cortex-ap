//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief main program
///
// Change: attitude control moved to inline static function
//         added remote tuning of Ki, Kp coefficients with two radio channels
//         Ki, Kp values stored to log files when exiting tuning mode
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "math.h"

#include "stm32f10x.h"

#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"
#include "adxl345_driver.h"
#include "servodriver.h"
#include "ppmdriver.h"
#include "diskio.h"

#include "config.h"
#include "dcm.h"
#include "nav.h"
#include "telemetry.h"
#include "log.h"
#include "led.h"
#include "pid.h"

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

/* Task priorities. */
#define mainAHRS_PRIORITY       ( tskIDLE_PRIORITY + 4 )
#define mainDISK_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainATTITUDE_PRIORITY   ( tskIDLE_PRIORITY + 3 )
#define mainLOG_PRIORITY        ( tskIDLE_PRIORITY + 2 )
#define mainTELEMETRY_PRIORITY  ( tskIDLE_PRIORITY + 2 )
#define mainNAVIGATION_PRIORITY ( tskIDLE_PRIORITY + 1 )

#define LOG_SENSORS       0
#define LOG_DCM           0
#define LOG_PPM           0
#define LOG_SERVO         0

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

VAR_STATIC const int16_t Sensor_Sign[6] = {
    -1,     // acceleration X, must be positive forward
     1,     // acceleration Y, must be positive rightward
     1,     // acceleration Z, must be positive downward
     1,	    // roll rate, must be positive when right wing lowers
    -1,     // pitch rate, must be positive when tail lowers
    -1      // yaw rate, must be positive when turning right
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC bool bTuning = FALSE;
VAR_STATIC int16_t iRudder;
VAR_STATIC int16_t iElevator;
VAR_STATIC uint8_t ucBlink_Red = 0;
VAR_STATIC uint8_t ucBlink_Blue = 0;
VAR_STATIC uint8_t ucSensor_Data[16];
VAR_STATIC int16_t iMessage_Buffer[9];
VAR_STATIC int16_t iSensor_Offset[6] = {0, 0, 0, 0, 0, 0};
VAR_STATIC xPID Roll_Pid;
VAR_STATIC xPID Pitch_Pid;
VAR_STATIC xLog_Message message;
VAR_STATIC float fSetpoint;
VAR_STATIC float fInput;
VAR_STATIC float fOutput;

/*--------------------------------- Prototypes -------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void AHRS_Task(void *pvParameters);
static __inline void Attitude_Control(void);

/*--------------------------------- Functions --------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   hook for stack overflow check
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void vApplicationStackOverflowHook( xTaskHandle *pxTask, int8_t *pcTaskName ) {
    ( void ) pxTask;
    ( void ) pcTaskName;
    for ( ;; );
}

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
       system_stm32f10x.c file */

  // Configure priority group
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // System Clocks Configuration
  RCC_Configuration();

  // GPIO Configuration
  GPIO_Configuration();

  // Initialize PWM timers as servo outputs
  Servo_Init();

  // Initialize capture timers as RRC input
  PPM_Init();

  // I2C peripheral initialization
  I2C_MEMS_Init();

  Telemetry_Init();

  xTelemetry_Queue = xQueueCreate( 3, sizeof( xTelemetry_Message ) );
  while ( xTelemetry_Queue == 0 ) {
  }

  xLog_Queue = xQueueCreate( 3, sizeof( xLog_Message ) );
  while ( xLog_Queue == 0 ) {
  }

  xGps_Queue = xQueueCreate( 3, sizeof( xGps_Message ) );
  while ( xGps_Queue == 0 ) {
  }

  xTaskCreate(AHRS_Task, ( signed portCHAR * ) "AHRS", 64, NULL, mainAHRS_PRIORITY, NULL);
//  xTaskCreate(Attitude_Control_Task, ( signed portCHAR * ) "Attitude", 64, NULL, mainATTITUDE_PRIORITY, NULL);
  xTaskCreate(disk_timerproc, ( signed portCHAR * ) "Disk", 32, NULL, mainDISK_PRIORITY, NULL);
//  xTaskCreate(Navigation_Task, ( signed portCHAR * ) "Navigation", 64, NULL, mainNAVIGATION_PRIORITY, NULL);
  xTaskCreate(Telemetry_Task, ( signed portCHAR * ) "Telemetry", 64, NULL, mainTELEMETRY_PRIORITY, NULL);
  xTaskCreate(Log_Task, ( signed portCHAR * ) "Log", 128, NULL, mainLOG_PRIORITY, NULL);

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

  /* TIM2 and TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

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

  // TIM2 Channel 2 as input pull down (1)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

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


///----------------------------------------------------------------------------
///
/// \brief   Attitude and heading computation.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void AHRS_Task(void *pvParameters)
{
    uint8_t i = 0, j = 0;
    int16_t * pSensor;
    portTickType Last_Wake_Time;

    Last_Wake_Time = xTaskGetTickCount();

    /* Task specific initializations */
    L3G4200_Init();                                 // init L3G4200 gyro
    ADXL345_Init();                                 // init ADXL345 accelerometer

    Roll_Pid.fGain = 500.0f;
    Roll_Pid.fMin = -1.0f;
    Roll_Pid.fMax = 1.0f;
    Roll_Pid.fKp = 1.0f;
    Roll_Pid.fKi = 0.0f;
    Roll_Pid.fKd = 0.0f;

    Pitch_Pid.fGain = 500.0f;
    Pitch_Pid.fMin = -1.0f;
    Pitch_Pid.fMax = 1.0f;
    Pitch_Pid.fKp = 1.0f;
    Pitch_Pid.fKi = 0.0f;
    Pitch_Pid.fKd = 0.0f;

    PID_Init(&Roll_Pid);
    PID_Init(&Pitch_Pid);

    /* Wait until aircraft settles */
    LEDOn(BLUE);
    vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ * 5);
    LEDOff(BLUE);

    /* Compute sensor offsets */
    for (i = 0; i < 64; i++) {
        vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ / SAMPLES_PER_SECOND);
        GetAccelRaw(ucSensor_Data);                   // acceleration
        GetAngRateRaw((uint8_t *)&ucSensor_Data[6]);  // angular rate
        pSensor = (int16_t *)ucSensor_Data;
        for (j = 0; j < 6; j++) {                   // accumulate
            iSensor_Offset[j] += *pSensor++;
        }
    }
    for (j = 0; j < 6; j++) {                       // average
        iSensor_Offset[j] = iSensor_Offset[j] / 64;
    }

    /* Compute attitude and heading */
    while (1) {
        vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ / SAMPLES_PER_SECOND);

        if (++ucBlink_Blue == 10) {                 // blink led every 10 cycles
           ucBlink_Blue = 0;
           LEDToggle(BLUE);
        }

        /* Read sensors */
        GetAccelRaw(ucSensor_Data);                 // acceleration
        GetAngRateRaw((uint8_t *)&ucSensor_Data[6]);// rotation

        /* Offset and sign correction */
        pSensor = (int16_t *)ucSensor_Data;
        for (j = 0; j < 6; j++) {
            *pSensor = *pSensor - iSensor_Offset[j];// strip offset
            *pSensor = *pSensor * Sensor_Sign[j];   // correct sign
            if (j == 2) {                           // z acceleration
               *pSensor += (int16_t)GRAVITY;        // add gravity
            }
            pSensor++;
        }

        /* AHRS */
        MatrixUpdate((int16_t *)ucSensor_Data);     // compute DCM
        CompensateDrift();                          // compensate
        Normalize();                                // normalize DCM
        Attitude_Control();                         // attitude control loop

        /* Log sensor data */
#if (LOG_SENSORS == 1)
        message.ucLength = 6;                       // message length
        message.pcData = (uint16_t *)ucSensor_Data; // message content
        xQueueSend( xLog_Queue, &message, 0 );
#endif
        /* Log PPM channels */
#if (LOG_PPM == 1)
        iMessage_Buffer[0] = (int16_t)PPMGetChannel(RUDDER_CHANNEL);
        iMessage_Buffer[1] = (int16_t)PPMGetChannel(ELEVATOR_CHANNEL);
        message.ucLength = 2;                        // message length
        message.pcData = (uint16_t *)iMessage_Buffer; // message content
        xQueueSend( xLog_Queue, &message, 0 );
#endif
        /* Log DCM matrix */
#if (LOG_DCM == 1)
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                iMessage_Buffer[(i * 3) + j] =
                (int16_t)(DCM_Matrix[i][j] * 32767.0f);
            }
        }
        message.ucLength = 9;                        // message length
        message.pcData = (uint16_t *)iMessage_Buffer; // message content
        xQueueSend( xLog_Queue, &message, 0 );
#endif
    }
}


///----------------------------------------------------------------------------
///
/// \brief   Attitude control.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
static __inline void Attitude_Control(void)
{
    iRudder = PPMGetChannel(RUDDER_CHANNEL);
    iElevator = PPMGetChannel(ELEVATOR_CHANNEL);

    switch (PPMGetMode()) {
        case MODE_PITCH_TUNE:
            bTuning = TRUE;
            Pitch_Pid.fKp = (float)(PPMGetChannel(KP_CHANNEL) - SERVO_MIN) / 250.0f;
            Pitch_Pid.fKi = (float)(PPMGetChannel(KI_CHANNEL) - SERVO_MIN) / 10000.0f;
            iElevator -= SERVO_NEUTRAL;
            fSetpoint = ((float)iElevator / 500.0f) + (PI/2);       // setpoint for pitch angle
            fInput = acosf(DCM_Matrix[2][0]);                       // pitch angle given by DCM
            fOutput = PID_Compute(&Pitch_Pid, fSetpoint, fInput);   // PID controller
            iElevator = SERVO_NEUTRAL + (int16_t)fOutput;
            if (++ucBlink_Red >= 4) {
                ucBlink_Red = 0;
                LEDToggle(RED);
            }
        break;
        case MODE_ROLL_TUNE:
            bTuning = TRUE;
            Roll_Pid.fKp = (float)(PPMGetChannel(KP_CHANNEL) - SERVO_MIN) / 250.0f;
            Roll_Pid.fKi = (float)(PPMGetChannel(KI_CHANNEL) - SERVO_MIN) / 10000.0f;
            iRudder -= SERVO_NEUTRAL;
            fSetpoint = ((float)iRudder / 500.0f) + (PI/2);         // setpoint for bank angle
            fInput = acosf(DCM_Matrix[2][1]);                       // bank angle given by DCM
            fOutput = PID_Compute(&Roll_Pid, fSetpoint, fInput);    // PID controller
            iRudder = SERVO_NEUTRAL + (int16_t)fOutput;
            if (++ucBlink_Red >= 8) {
                ucBlink_Red = 0;
                LEDToggle(RED);
            }
        break;
//       case MODE_MANUAL:
//       case MODE_RTL:
        default:
            PID_Init(&Roll_Pid);    // reset PID controllers
            PID_Init(&Pitch_Pid);
            LEDOff(RED);
            if (bTuning) {
                iMessage_Buffer[0] = (int16_t)(Roll_Pid.fKp * 1000.0f);
                iMessage_Buffer[1] = (int16_t)(Roll_Pid.fKi * 40000.0f);
                iMessage_Buffer[2] = (int16_t)(Pitch_Pid.fKp * 1000.0f);
                iMessage_Buffer[3] = (int16_t)(Pitch_Pid.fKi * 40000.0f);
                message.ucLength = 4;                         // message length
                message.pcData = (uint16_t *)iMessage_Buffer; // message content
                xQueueSend( xLog_Queue, &message, 0 );
                bTuning = FALSE;
            }
        break;
    }
    Servo_Set(SERVO_AILERON, iRudder);
    Servo_Set(SERVO_ELEVATOR, iElevator);
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
