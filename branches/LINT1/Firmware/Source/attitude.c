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
// Change: first Lint pass
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

VAR_STATIC bool b_Tuning = FALSE;        //!< in flight PID tuning activated
VAR_STATIC int16_t i_Aileron;            //!< aileron servo position
VAR_STATIC int16_t i_Elevator;           //!< elevator servo position
VAR_STATIC int16_t i_Throttle;           //!< throttle servo position
VAR_STATIC uint8_t uc_Blink_Red = 0;     //!< red LED blinking counter
VAR_STATIC uint8_t uc_Blink_Blue = 0;    //!< blue LED blinking counter
VAR_STATIC xPID Roll_Pid;                //!< roll PID
VAR_STATIC xPID Pitch_Pid;               //!< pitch PID
VAR_STATIC float f_Setpoint;             //!< PID setpoint
VAR_STATIC float f_Input;                //!< PID input
VAR_STATIC float f_Output;               //!< PID output
VAR_STATIC uint8_t uc_Sensor_Data[12];   //!< raw IMU data
VAR_STATIC int16_t i_Sensor_Offset[6] =  //!< IMU sensors offset
{ 0, 0, 0, 0, 0, 0 };

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
    int16_t * p_sensor;
    portTickType Last_Wake_Time;

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

    PID_Init(&Roll_Pid);
    PID_Init(&Pitch_Pid);

    /* Wait until aircraft settles */
    LEDOn(BLUE);
    vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ * 5);
    LEDOff(BLUE);

    /* Compute sensor offsets */
    for (i = 0; i < 64; i++) {
        vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ / SAMPLES_PER_SECOND);
#if (SIMULATOR == SIM_NONE)                                 // normal mode
        (void)GetAccelRaw(uc_Sensor_Data);                  // acceleration
        (void)GetAngRateRaw((uint8_t *)&uc_Sensor_Data[6]); // rotation
#else                                                       // simulation mode
        Simulator_Get_Raw_IMU((int16_t *)uc_Sensor_Data);   // get simulator sensors
#endif
        p_sensor = (int16_t *)uc_Sensor_Data;
        for (j = 0; j < 6; j++) {                       // accumulate
            i_Sensor_Offset[j] += *p_sensor++;
        }
    }
    for (j = 0; j < 6; j++) {                           // average
        i_Sensor_Offset[j] = i_Sensor_Offset[j] / 64;
    }

    /* Compute attitude and heading */
    for (;;) {

        vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ / SAMPLES_PER_SECOND);

        uc_Blink_Blue = (uc_Blink_Blue + 1) % 10;       // blink led @ 10 Hz
        if (uc_Blink_Blue < 3) {                        // duty cycle 30 %
            LEDOn(BLUE);
        } else {
            LEDOff(BLUE);
        }

        /* Read sensors */
#if (SIMULATOR == SIM_NONE)                                   // normal mode
        (void)GetAccelRaw(uc_Sensor_Data);                    // acceleration
        (void)GetAngRateRaw((uint8_t *)&uc_Sensor_Data[6]);   // rotation
        BMP085_Handler();
#else                                                         // simulation mode
        Simulator_Get_Raw_IMU((int16_t *)uc_Sensor_Data);     // get simulator sensors
#endif
        /* Offset and sign correction */
        p_sensor = (int16_t *)uc_Sensor_Data;
        for (j = 0; j < 6; j++) {
            *p_sensor -= i_Sensor_Offset[j];            // strip offset
            *p_sensor *= iSensor_Sign[j];               // correct sign
            if (j == 2) {                               // z acceleration
               *p_sensor += (int16_t)GRAVITY;           // add gravity
            }
            p_sensor++;
        }

        /* AHRS */
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
#if (SIMULATOR == SIM_NONE)
    Pitch_Pid.fKp = Telemetry_Get_Gain(TEL_PITCH_KP);
    Pitch_Pid.fKi = Telemetry_Get_Gain(TEL_PITCH_KI);
    Roll_Pid.fKp = Telemetry_Get_Gain(TEL_ROLL_KP);
    Roll_Pid.fKi = Telemetry_Get_Gain(TEL_ROLL_KI);
#else
    Pitch_Pid.fKp = Simulator_Get_Gain(SIM_PITCH_KP);
    Pitch_Pid.fKi = Simulator_Get_Gain(SIM_PITCH_KI);
    Roll_Pid.fKp = Simulator_Get_Gain(SIM_ROLL_KP);
    Roll_Pid.fKi = Simulator_Get_Gain(SIM_ROLL_KI);
#endif
    switch (PPMGetMode()) {

        case MODE_STAB:                                             // STABILIZED MODE
            i_Aileron = PPMGetChannel(AILERON_CHANNEL) - SERVO_NEUTRAL;
            i_Elevator = PPMGetChannel(ELEVATOR_CHANNEL) - SERVO_NEUTRAL;
            i_Throttle = SERVO_NEUTRAL + (int16_t)(500.0f * Nav_Throttle());

            f_Setpoint = ((float)i_Elevator / 500.0f);                // setpoint for pitch
            f_Input = -asinf(DCM_Matrix[2][0]);                      // current pitch
            f_Output = PID_Compute(&Pitch_Pid, f_Setpoint, f_Input);   // PID controller
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Output;

            f_Setpoint = ((float)i_Aileron / 500.0f);                 // setpoint for bank
            f_Input =  -asinf(DCM_Matrix[2][1]);                     // current bank
            f_Output = PID_Compute(&Roll_Pid, f_Setpoint, f_Input);    // PID controller

            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Output;

            if (++uc_Blink_Red >= 8) {                               // slow blink
                uc_Blink_Red = 0;
                LEDToggle(RED);
            }
            break;

        case MODE_NAV:                                              // NAVIGATION MODE
            f_Setpoint = Nav_Pitch_Rad();                            // setpoint for pitch
            f_Input = -asinf(DCM_Matrix[2][0]);                      // current pitch
            f_Output = PID_Compute(&Pitch_Pid, f_Setpoint, f_Input);   // PID controller
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Output;

            f_Setpoint = Nav_Bank_Rad();                             // setpoint for bank
            f_Input = -asinf(DCM_Matrix[2][1]);                      // current bank
            f_Output = PID_Compute(&Roll_Pid, f_Setpoint, f_Input);    // PID controller
            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Output;

            i_Throttle = SERVO_NEUTRAL + (int16_t)(500.0f * Nav_Throttle());

            if (++uc_Blink_Red >= 4) {                               // fast blink
                uc_Blink_Red = 0;
                LEDToggle(RED);
            }
            break;

        case MODE_FPV:                                              // CAMERA STABILIZATION MODE
            f_Input = -(Attitude_Pitch_Rad() * 1800.0f) / PI;        // pitch angle given by DCM
            i_Elevator = SERVO_NEUTRAL + (int16_t)f_Input;            // show on elevator servo
            f_Input = -(Attitude_Roll_Rad() * 1800.0f) / PI;         // bank angle given by DCM
            i_Aileron = SERVO_NEUTRAL + (int16_t)f_Input;             // show on aileron servo
            break;

//       case MODE_MANUAL:
//       case MODE_RTL:
        default:                                                    // MANUAL MODE
            i_Aileron = PPMGetChannel(AILERON_CHANNEL);
            i_Elevator = PPMGetChannel(ELEVATOR_CHANNEL);
            i_Throttle = PPMGetChannel(THROTTLE_CHANNEL);
            PID_Init(&Roll_Pid);                                    // reset PID controllers
            PID_Init(&Pitch_Pid);
            LEDOff(RED);
            if (b_Tuning) {
                b_Tuning = FALSE;
            }
            break;
    }

    /* Update controls */
    Servo_Set(SERVO_AILERON, i_Aileron);                             // update servos
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
