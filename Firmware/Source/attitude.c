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
///  Reference point for pitch and roll stabilization is computed as
///     -asinf(DCM[...][...])
///  insetad of
///     acosf(DCM[...][...]) - PI/2
///  This returns an angle value that's consistent with angle convention for
///  roll and pitch angles, without need to either subtract PI/2 from reference
///  point or to add PI/2 to the set point.
///
// Change: restored sensor calibration
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"

#include "stm32f10x.h"
#include "stm32f10x_wwdg.h"

#include "i2c_mems_driver.h"
#include "l3g4200d_driver.h"
#include "adxl345_driver.h"
#include "bmp085_driver.h"
#include "servodriver.h"
#include "ppmdriver.h"

#include "config.h"
#include "dcm.h"
#include "simulator.h"
#include "mav_telemetry.h"
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

#ifndef VAR_STATIC
#define VAR_STATIC static
#endif

/* delay for attitude task */
#define AHRS_DELAY      (configTICK_RATE_HZ / SAMPLES_PER_SECOND)

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

VAR_STATIC const uint8_t uc_Blink[MODE_NUM][100] = {
{ 0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 },
{ 1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 },
{ 1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 },
{ 1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 },
{ 1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 },
{ 1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  1,1,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0 }
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC bool b_Tuning = FALSE;        //!< in flight PID tuning activated
VAR_STATIC int16_t i_Aileron;            //!< aileron servo position
VAR_STATIC int16_t i_Elevator;           //!< elevator servo position
VAR_STATIC int16_t i_Throttle;           //!< throttle servo position
VAR_STATIC uint8_t uc_Counter = 0;       //!< blue LED blinking counter
VAR_STATIC xPID Roll_Pid;                //!< roll PID
VAR_STATIC xPID Pitch_Pid;               //!< pitch PID
VAR_STATIC xPID Nav_Pid;                 //!< navigation PID
VAR_STATIC float f_Setpoint;             //!< PID setpoint
VAR_STATIC float f_Input;                //!< PID input
VAR_STATIC float f_Output;               //!< PID output
VAR_STATIC float f_Pitch = 0.0f;         //!< commanded pitch
VAR_STATIC float f_Height_Margin = HEIGHT_MARGIN;        //!< altitude hold margin
VAR_STATIC float f_Throttle = MINIMUMTHROTTLE;           //!< commanded throttle
VAR_STATIC float f_Throttle_Min = ALT_HOLD_THROTTLE_MIN; //!< altitude hold min throttle
VAR_STATIC float f_Throttle_Max = ALT_HOLD_THROTTLE_MAX; //!< altitude hold max throttle
VAR_STATIC uint8_t uc_Sensor_Data[12];   //!< raw sensor data
VAR_STATIC int16_t i_Sensor_Offset[6] =  //!< sensors offset
{ 0x02, 0x00, 0x3F, 0x02, 0x05, 0x06 };

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
    uint8_t i, j;
    int16_t * p_sensor;
    portTickType Last_Wake_Time;

    (void)pvParameters;
    Last_Wake_Time = xTaskGetTickCount();

    /* Task specific initializations */
    L3G4200_Init();                                 // init L3G4200 gyro
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

    Nav_Pid.fGain = ToRad(NAV_BANK);                // limit bank angle during navigation
    Nav_Pid.fMin = -1.0f;                           //
    Nav_Pid.fMax = 1.0f;                            //
    Nav_Pid.fKp = NAV_KP;                           // init gains with default values
    Nav_Pid.fKi = NAV_KI;                           //
    Nav_Pid.fKd = NAV_KD;                           //

    PID_Init(&Roll_Pid);                            // initialize PID
    PID_Init(&Pitch_Pid);
    PID_Init(&Nav_Pid);

    /* Wait until aircraft settles */
    LEDOn(BLUE);
    vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ * 5);
    LEDOff(BLUE);

    /* Compute sensor offsets */
    for (i = 0; i < 64; i++) {
        vTaskDelayUntil(&Last_Wake_Time, AHRS_DELAY);
  #if (SIMULATOR == SIM_NONE)                               // normal mode
        (void)GetAccelRaw(uc_Sensor_Data);                  // acceleration
        (void)GetAngRateRaw((uint8_t *)&uc_Sensor_Data[6]); // rotation
  #else                                                     // simulation mode
        Simulator_Get_Raw_IMU((int16_t *)uc_Sensor_Data);   // get simulator sensors
  #endif
        p_sensor = (int16_t *)uc_Sensor_Data;
        for (j = 0; j < 6; j++) {                           // accumulate
            i_Sensor_Offset[j] += *p_sensor++;
        }
    }
    for (j = 0; j < 6; j++) {                               // average
        i_Sensor_Offset[j] = i_Sensor_Offset[j] / 64;
    }

    for (;;) {                                              // endless loop
        vTaskDelayUntil(&Last_Wake_Time, AHRS_DELAY);       // update @ 50 Hz
        WWDG_SetCounter(127);                               // update WWDG counter
        uc_Counter = (uc_Counter + 1) % 100;                // blink blue LED
        if (uc_Blink[PPMGetMode()][uc_Counter] == 1) {      //
            LEDOn(BLUE);
        } else {
            LEDOff(BLUE);
        }

        /* Read sensors */
#if (SIMULATOR == SIM_NONE)                                 // normal mode
        (void)GetAccelRaw(uc_Sensor_Data);                  // acceleration
        (void)GetAngRateRaw((uint8_t *)&uc_Sensor_Data[6]); // rotation
        BMP085_Handler();
#else                                                       // simulation mode
        Simulator_Get_Raw_IMU((int16_t *)uc_Sensor_Data);   // get simulator sensors
#endif
        /* Offset and sign correction */
        p_sensor = (int16_t *)uc_Sensor_Data;
        for (j = 0; j < 6; j++) {
            *p_sensor -= i_Sensor_Offset[j];                // strip offset
            *p_sensor *= iSensor_Sign[j];                   // correct sign
            if (j == 2) {                                   // z acceleration
               *p_sensor += (int16_t)GRAVITY;               // add gravity
            }
            p_sensor++;
        }

        /* AHRS and control */
        MatrixUpdate((int16_t *)uc_Sensor_Data);        // compute DCM
        CompensateDrift();                              // compensate
        Normalize();                                    // normalize DCM
        Attitude_Control();                             // attitude control loop
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
    float f_temp;
    /* update PID gains */
#if (SIMULATOR == SIM_NONE)
    Pitch_Pid.fKp = Telemetry_Get_Gain(TEL_PITCH_KP);
    Pitch_Pid.fKi = Telemetry_Get_Gain(TEL_PITCH_KI);
    Roll_Pid.fKp = Telemetry_Get_Gain(TEL_ROLL_KP);
    Roll_Pid.fKi = Telemetry_Get_Gain(TEL_ROLL_KI);
    Nav_Pid.fKp = Telemetry_Get_Gain(TEL_NAV_KP);
    Nav_Pid.fKi = Telemetry_Get_Gain(TEL_NAV_KI);
    Nav_Pid.fGain = Telemetry_Get_Gain(TEL_NAV_BANK);
#else
    Pitch_Pid.fKp = Simulator_Get_Gain(SIM_PITCH_KP);
    Pitch_Pid.fKi = Simulator_Get_Gain(SIM_PITCH_KI);
    Roll_Pid.fKp = Simulator_Get_Gain(SIM_ROLL_KP);
    Roll_Pid.fKi = Simulator_Get_Gain(SIM_ROLL_KI);
    Nav_Pid.fKp = Simulator_Get_Gain(SIM_NAV_KP);
    Nav_Pid.fKi = Simulator_Get_Gain(SIM_NAV_KI);
#endif

    switch (PPMGetMode()) {

        case MODE_STAB:                                                 // STABILIZED MODE
            i_Aileron = PPMGetChannel(AILERON_CHANNEL) - SERVO_NEUTRAL;
            i_Elevator = PPMGetChannel(ELEVATOR_CHANNEL) - SERVO_NEUTRAL;
            /* throttle control */
            f_temp = Nav_Alt_Error();                                   // altitude error
            if (f_temp > f_Height_Margin) {                             // above maximum
                f_Throttle = f_Throttle_Min;                            // minimum throttle
                f_Pitch = PITCHATMINTHROTTLE;                           // pitch accordingly
            } else if (f_temp < - f_Height_Margin) {                    // below minimum
                f_Throttle = f_Throttle_Max;                            // max throttle
                f_Pitch = PITCHATMAXTHROTTLE;                           // pitch accordingly
            } else {                                                    // interpolate
                f_temp = (f_temp - HEIGHT_MARGIN) / (2.0f * HEIGHT_MARGIN);
                f_Throttle = f_temp * (f_Throttle_Min - f_Throttle_Max) + f_Throttle_Min;
                f_Pitch = f_temp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
            }
            i_Throttle = SERVO_NEUTRAL + (int16_t)(500.0f * f_Throttle);
            /* pitch control */
            f_Setpoint = ((float)i_Elevator / 500.0f);                  // setpoint for pitch
            f_Input = -asinf(DCM_Matrix[2][0]);                         // current pitch
            f_Output = PID_Compute(&Pitch_Pid, f_Setpoint, f_Input);    // pitch PID
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Output;
            /* roll control */
            f_Setpoint = ((float)i_Aileron / 500.0f);                   // setpoint for bank
            f_Input =  -asinf(DCM_Matrix[2][1]);                        // current bank
            f_Output = PID_Compute(&Roll_Pid, f_Setpoint, f_Input);     // rol PID
            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Output;
            break;

        case MODE_NAV:                                                  // NAVIGATION MODE
            /* bank control */
            f_Setpoint = Nav_Dir_Error();                               // direction error
            f_Setpoint = PID_Compute(&Nav_Pid, f_Setpoint, 0.0f);       // direction PID
            /* roll control */
            f_Input = -asinf(DCM_Matrix[2][1]);                         // current bank
            f_Output = PID_Compute(&Roll_Pid, f_Setpoint, f_Input);     // roll PID
            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Output;
            /* throttle control */
            f_temp = Nav_Alt_Error();                                   // altitude error
            if (f_temp > f_Height_Margin) {                             // above maximum
                f_Throttle = f_Throttle_Min;                            // minimum throttle
                f_Pitch = PITCHATMINTHROTTLE;
            } else if (f_temp < - f_Height_Margin) {                    // below minimum
                f_Throttle = f_Throttle_Max;                            // max throttle
                f_Pitch = PITCHATMAXTHROTTLE;
            } else {                                                    // interpolate
                f_temp = (f_temp - HEIGHT_MARGIN) / (2.0f * HEIGHT_MARGIN);
                f_Throttle = f_temp * (f_Throttle_Min - f_Throttle_Max) + f_Throttle_Min;
                f_Pitch = f_temp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
            }
            i_Throttle = SERVO_NEUTRAL + (int16_t)(500.0f * f_Throttle);
            /* pitch control */
            f_Setpoint = f_Pitch;                                       // pitch setpoint
            f_Input = -asinf(DCM_Matrix[2][0]);                         // current pitch
            f_Output = PID_Compute(&Pitch_Pid, f_Setpoint, f_Input);    // pitch PID
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Output;
            break;

        case MODE_FPV:                                                  // CAMERA STABILIZATION MODE
            f_Input = -(Attitude_Pitch_Rad() * 1800.0f) / PI;           // pitch angle given by DCM
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Input;              // show on elevator servo
            f_Input = -(Attitude_Roll_Rad() * 1800.0f) / PI;            // bank angle given by DCM
            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Input;               // show on aileron servo
            break;

//       case MODE_MANUAL:
//       case MODE_RTL:
        default:                                                        // MANUAL MODE
            i_Aileron = PPMGetChannel(AILERON_CHANNEL);
            i_Elevator = PPMGetChannel(ELEVATOR_CHANNEL);
            i_Throttle = PPMGetChannel(THROTTLE_CHANNEL);
            PID_Init(&Roll_Pid);                                        // reset PID controllers
            PID_Init(&Pitch_Pid);
            PID_Init(&Nav_Pid);
            if (b_Tuning) {
                b_Tuning = FALSE;
            }
            break;
    }

    /* Update controls */
    Servo_Set(SERVO_AILERON, i_Aileron);                                // update servos
    Servo_Set(SERVO_ELEVATOR, i_Elevator);
    Servo_Set(SERVO_THROTTLE, i_Throttle);
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft pitch.
/// \return  aircraft pitch angle [deg]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Pitch_Deg(void)
{
    return (ToDeg(-asinf(DCM_Matrix[2][0])));
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft roll.
/// \return  aircraft roll angle [deg]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Roll_Deg(void)
{
    return (ToDeg(asinf(DCM_Matrix[2][1])));
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft roll.
/// \return  aircraft roll angle [deg]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Yaw_Deg(void)
{
    return (ToDeg(atan2f(DCM_Matrix[1][0], DCM_Matrix[0][0])));
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft pitch.
/// \return  aircraft pitch angle [rad]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Pitch_Rad(void)
{
    return (-asinf(DCM_Matrix[2][0]));
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft roll.
/// \return  aircraft roll angle [rad]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Roll_Rad(void)
{
    return (asinf(DCM_Matrix[2][1]));
}

///----------------------------------------------------------------------------
///
/// \brief   Aircraft yaw.
/// \return  aircraft roll angle [rad]
/// \remarks -
///
///----------------------------------------------------------------------------
float Attitude_Yaw_Rad(void)
{
    return atan2f(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}


/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
