//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Direction Cosine Matrix calculations
///
/// \file
///
//  CHANGES removed minor defects detectd by static analysis
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

VAR_GLOBAL float DCM_Matrix[3][3] ;
VAR_GLOBAL float Gyro_Vector[3] ;
VAR_GLOBAL float Omega_Vector[3] ;
VAR_GLOBAL float Gyro_Gain ;
VAR_GLOBAL float Accel_Gain ;
VAR_GLOBAL float PitchRoll_Kp ;
VAR_GLOBAL float PitchRoll_Ki ;
VAR_GLOBAL float Yaw_Kp ;
VAR_GLOBAL float Yaw_Ki ;
VAR_GLOBAL float speed_3d ;

/*---------------------------------- Interface -------------------------------*/

void Normalize( void );
void CompensateDrift( void );
void AccelAdjust( void );
void MatrixUpdate( int16_t * sensor );
