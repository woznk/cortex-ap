//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief attitude control
///
/// \file
///
// Change: size of ucSensor_Data[] reduced to 12 bytes, 
//         function Telemetry_Get_Sensors() renamed Telemetry_Get_Raw_IMU()
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
#include "bmp085_driver.h"
#include "servodriver.h"
#include "ppmdriver.h"

#include "config.h"
#include "dcm.h"
#include "telemetry.h"
#include "log.h"
#include "led.h"
#include "nav.h"
#include "pid.h"
#include "attitude.h"

/** @addtogroup cortex_ap
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

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/// sign of sensor data
VAR_STATIC const int16_t iSensor_Sign[6] = {
    -1,     // acceleration X, must be positive forward
     1,     // acceleration Y, must be positive rightward
     1,     // acceleration Z, must be positive downward
     1,	    // roll rate, must be positive when right wing lowers
    -1,     // pitch rate, must be positive when tail lowers
    -1      // yaw rate, must be positive when turning right
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC bool bTuning = FALSE;        //!< in flight PID tuning activated
VAR_STATIC int16_t iAileron;            //!< aileron servo position
VAR_STATIC int16_t iElevator;           //!< elevator servo position
VAR_STATIC int16_t iThrottle;           //!< throttle servo position
VAR_STATIC uint8_t ucBlink_Red = 0;     //!< red LED blinking counter
VAR_STATIC uint8_t ucBlink_Blue = 0;    //!< blue LED blinking counter
VAR_STATIC xPID Roll_Pid;               //!< roll PID
VAR_STATIC xPID Pitch_Pid;              //!< pitch PID
VAR_STATIC float fSetpoint;             //!< PID setpoint
VAR_STATIC float fInput;                //!< PID input
VAR_STATIC float fOutput;               //!< PID output
VAR_STATIC uint8_t ucSensor_Data[12];   //!< raw IMU data
VAR_STATIC int16_t iSensor_Offset[6] =  //!< IMU sensors offset
{0, 0, 0, 0, 0, 0};

/*--------------------------------- Prototypes -------------------------------*/

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
    ( void )L3G4200_Init();                         // init L3G4200 gyro
    ( void )ADXL345_Init();                         // init ADXL345 accelerometer
    ( void )BMP085_Init();

    Roll_Pid.fGain = 500.0f;                        // limit servo throw
    Roll_Pid.fMin = -1.0f;                          //
    Roll_Pid.fMax = 1.0f;                           //
    Roll_Pid.fKp = ROLL_KP;                         // init gains with default values
    Roll_Pid.fKi = ROLL_KI;                         //
    Roll_Pid.fKd = ROLL_KD;                         //

    Pitch_Pid.fGain = 500.0f;                       // limit servo throw
    Pitch_Pid.fMin = -1.0f;                         //
    Pitch_Pid.fMax = 1.0f;                          //
    Pitch_Pid.fKp = PITCH_KP;                       // init gains with default values
    Pitch_Pid.fKi = PITCH_KI;                       //
    Pitch_Pid.fKd = PITCH_KD;                       //

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
        GetAngRateRaw((uint8_t *)&ucSensor_Data[6]); // rotation
#else                                               // simulation mode
        Telemetry_Get_Raw_IMU((int16_t *)ucSensor_Data);// get simulator sensors
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
        BMP085_Handler();
#else                                               // simulation mode
        Telemetry_Get_Raw_IMU((int16_t *)ucSensor_Data);// get simulator sensors
#endif
        /* Offset and sign correction */
        pSensor = (int16_t *)ucSensor_Data;
        for (j = 0; j < 6; j++) {
            *pSensor = *pSensor - iSensor_Offset[j];// strip offset
            *pSensor = *pSensor * iSensor_Sign[j];  // correct sign
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
    Pitch_Pid.fKp = Telemetry_Get_Gain(TEL_PITCH_KP);
    Pitch_Pid.fKi = Telemetry_Get_Gain(TEL_PITCH_KI);
    Roll_Pid.fKp = Telemetry_Get_Gain(TEL_ROLL_KP);
    Roll_Pid.fKi = Telemetry_Get_Gain(TEL_ROLL_KI);

    switch (PPMGetMode()) {

        case MODE_STAB:
            iAileron = PPMGetChannel(AILERON_CHANNEL) - SERVO_NEUTRAL;
            iElevator = PPMGetChannel(ELEVATOR_CHANNEL) - SERVO_NEUTRAL;
            iThrottle = SERVO_NEUTRAL + (int16_t)(500.0f * Nav_Throttle());

            fSetpoint = ((float)iElevator / 500.0f) + (PI / 2.0f);  // setpoint for pitch angle
            fInput = acosf(DCM_Matrix[2][0]);                       // pitch angle given by DCM
            fOutput = PID_Compute(&Pitch_Pid, fSetpoint, fInput);   // PID controller
            iElevator = SERVO_NEUTRAL + (int16_t)fOutput;

            fSetpoint = ((float)iAileron / 500.0f) + (PI / 2.0f);   // setpoint for bank angle
            fInput = acosf(DCM_Matrix[2][1]);                       // bank angle given by DCM
            fOutput = PID_Compute(&Roll_Pid, fSetpoint, fInput);    // PID controller
            iAileron = SERVO_NEUTRAL + (int16_t)fOutput;

            if (++ucBlink_Red >= 8) {
                ucBlink_Red = 0;
                LEDToggle(RED);
            }
            break;

        case MODE_NAV:
            fSetpoint = Nav_Pitch();                                // setpoint for pitch angle
            fInput = acosf(DCM_Matrix[2][0]);                       // pitch angle given by DCM
            fOutput = PID_Compute(&Pitch_Pid, fSetpoint, fInput);   // PID controller
            iElevator = SERVO_NEUTRAL + (int16_t)fOutput;

            fSetpoint = Nav_Bank();                                 // setpoint for bank angle
            fInput = acosf(DCM_Matrix[2][1]);                       // bank angle given by DCM
            fOutput = PID_Compute(&Roll_Pid, fSetpoint, fInput);    // PID controller
            iAileron = SERVO_NEUTRAL + (int16_t)fOutput;

            iThrottle = SERVO_NEUTRAL + (int16_t)(500.0f * Nav_Throttle());

            if (++ucBlink_Red >= 4) {
                ucBlink_Red = 0;
                LEDToggle(RED);
            }
            break;

//       case MODE_MANUAL:
//       case MODE_RTL:
        default:
            iAileron = PPMGetChannel(AILERON_CHANNEL);
            iElevator = PPMGetChannel(ELEVATOR_CHANNEL);
            iThrottle = PPMGetChannel(THROTTLE_CHANNEL);
            PID_Init(&Roll_Pid);                                    // reset PID controllers
            PID_Init(&Pitch_Pid);
            LEDOff(RED);
            if (bTuning) {
                bTuning = FALSE;
            }
            break;
    }
    /* Update controls */
    Servo_Set(SERVO_AILERON, iAileron);                             // update servos
    Servo_Set(SERVO_ELEVATOR, iElevator);
    Servo_Set(SERVO_THROTTLE, iThrottle);
}

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
