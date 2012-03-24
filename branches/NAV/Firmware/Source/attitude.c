//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief attitude control
///
// Change: corrected reading of sensor data from simulator 
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"

#include "stm32f10x.h"

#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"
#include "adxl345_driver.h"
#include "servodriver.h"
#include "ppmdriver.h"

#include "config.h"
#include "dcm.h"
#include "telemetry.h"
#include "log.h"
#include "led.h"
#include "pid.h"

/** @addtogroup cortex-ap
  * @{
  */

/** @addtogroup attitude
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

void AHRS_Task(void *pvParameters);
static __inline void Attitude_Control(void);

/*--------------------------------- Functions --------------------------------*/


///----------------------------------------------------------------------------
///
/// \brief   Aircraft attitude computation and control.
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Attitude_Task(void *pvParameters)
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
#if (SIMULATOR == SIM_NONE)                         // normal mode
        GetAccelRaw(ucSensor_Data);                 // acceleration
        GetAngRateRaw((uint8_t *)&ucSensor_Data[6]);// rotation
#else                                               // simulation mode
        Telemetry_Get_Sensors((int16_t *)ucSensor_Data);// get simulator sensors
#endif
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
#if (SIMULATOR == SIM_NONE)                         // normal mode
        GetAccelRaw(ucSensor_Data);                 // acceleration
        GetAngRateRaw((uint8_t *)&ucSensor_Data[6]);// rotation
#else                                               // simulation mode
        Telemetry_Get_Sensors((int16_t *)ucSensor_Data);// get simulator sensors
#endif
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
        iMessage_Buffer[0] = (int16_t)PPMGetChannel(AILERON_CHANNEL);
        iMessage_Buffer[1] = (int16_t)PPMGetChannel(ELEVATOR_CHANNEL);
        iMessage_Buffer[2] = (int16_t)PPMGetChannel(MODE_CHANNEL);
        iMessage_Buffer[3] = (int16_t)PPMGetChannel(THROTTLE_CHANNEL);
        iMessage_Buffer[4] = (int16_t)PPMGetChannel(RUDDER_CHANNEL);

        message.ucLength = 5;                        // message length
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
    iRudder = PPMGetChannel(AILERON_CHANNEL);
    iElevator = PPMGetChannel(ELEVATOR_CHANNEL);

    switch (PPMGetMode()) {
        case MODE_PITCH_TUNE:
            bTuning = TRUE;
            Pitch_Pid.fKp = Telemetry_Get_Gain(TEL_PITCH_KP);
            Pitch_Pid.fKi = Telemetry_Get_Gain(TEL_PITCH_KI);
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
            Roll_Pid.fKp = Telemetry_Get_Gain(TEL_ROLL_KP);
            Roll_Pid.fKi = Telemetry_Get_Gain(TEL_ROLL_KI);
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
            PID_Init(&Roll_Pid);            // reset PID controllers
            PID_Init(&Pitch_Pid);
            LEDOff(RED);
            if (bTuning) {
                bTuning = FALSE;
            }
        break;
    }
#if (SIMULATOR == SIM_NONE)                 // normal mode
    Servo_Set(SERVO_AILERON, iRudder);      // update servos
    Servo_Set(SERVO_ELEVATOR, iElevator);
#else                                       // simulation mode
    Telemetry_Send_Controls();              // update simulator controls
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
