//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Direction Cosine Matrix calculations header file
///
/// \file
///
//  Change (Lint) argument sensor of MatriUxpdate made const int16_t *
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float DCM_Matrix[3][3] ; //!< Direction Cosine Matrix
VAR_GLOBAL float Gyro_Vector[3] ;   //!< Raw gyroscope data
VAR_GLOBAL float Omega_Vector[3] ;  //!< g-corrected gyroscope data
VAR_GLOBAL float Gyro_Gain ;        //!< Conversion gain from ADC to angular speed in deg/s
VAR_GLOBAL float Accel_Gain ;       //!< Conversion gain from ADC to acceleration in m/s/s
VAR_GLOBAL float PitchRoll_Kp ;     //!< Proportional gain roll/pitch compensation
VAR_GLOBAL float PitchRoll_Ki ;     //!< Integral gain roll/pitch compensation
VAR_GLOBAL float Yaw_Kp ;           //!< Proportional gain yaw compensation
VAR_GLOBAL float Yaw_Ki ;           //!< Integral gain yaw compensation
VAR_GLOBAL float fGround_Speed ;    //!< Velocity 3D

/*---------------------------------- Interface -------------------------------*/

void Normalize( void );
void CompensateDrift( void );
void AccelAdjust( void );
void MatrixUpdate( const int16_t * sensor );

