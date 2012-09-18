/*============================================================================*/
/* SFUNCTION NUMERICAL CONTROLLER                                             */
/*============================================================================*/
/* Authors: Paul BIZARD / DIY Drones                                          */
/* Date   : April 2009                                                        */
/*============================================================================*/
/* INPUTS                                                                     */
/* ------                                                                     */
/* u0[0 1 2] Gyrometers p q r       rad/s                                     */
/* u1[0 1 2] Accelerometers x y z   m/s^2                                     */
/* u2[0 1]   GPS speed over ground  m/s                                       */
/* u3[0]     GPS course over ground rad                                       */
/*============================================================================*/
/* OUTPUTS                                                                    */
/* -------                                                                    */
/* y0[0 1 2] Euler angles           rad                                       */
/* y1[0 1 2]                                                                  */
/* y2[0 1 2]                                                                  */
/* y3[0 1 2]                                                                  */
/* y4[0 1 2]                                                                  */
/* y5[0 1 2]                                                                  */
/*============================================================================*/
/* EKF attitude estimation                                                    */
/*                                                                            */
/* R marix = transformation from body frame to earth frame                    */
/*                                                                            */
/* 1st column is x body vector expressed in earth frame                       */
/* 2nd...        y                                                            */
/* 3rd...        z                                                            */
/*                                                                            */
/* 1st row is x earth vector expressed in body frame                          */
/* 2nd row    y                                                               */
/* 3rd row    z                                                               */
/*                                                                            */
/* x front                                                                    */
/* y right wing                                                               */
/* z down                                                                     */
/*============================================================================*/

#define S_FUNCTION_NAME       EKF
#define S_FUNCTION_LEVEL      2
#define NUM_INPUTS            4
#define INPUT_0_WIDTH         3
#define INPUT_1_WIDTH         3
#define INPUT_2_WIDTH         2
#define INPUT_3_WIDTH         1
#define INPUT_0_FEEDTHROUGH   1
#define INPUT_1_FEEDTHROUGH   1
#define INPUT_2_FEEDTHROUGH   1
#define INPUT_3_FEEDTHROUGH   1
#define NUM_OUTPUTS           6
#define OUTPUT_0_WIDTH        3
#define OUTPUT_1_WIDTH        3
#define OUTPUT_2_WIDTH        3
#define OUTPUT_3_WIDTH        3
#define OUTPUT_4_WIDTH        3
#define OUTPUT_5_WIDTH        3
#define NPARAMS               0
#define SAMPLE_TIME_0         0.025
#define SAMPLE_TIME_1         1.  
#define NUM_DISC_STATES       0
#define DISC_STATES_IC        [0]
#define NUM_CONT_STATES       0
#define CONT_STATES_IC        [0]
#define SFUNWIZ_GENERATE_TLC  1
#define SOURCEFILES           ""
#define PANELINDEX            5
#define SFUNWIZ_REVISION      1.0

#include "simstruc.h"

#include "conio.h"
#include "stdio.h"
#include "dos.h"
#include "math.h"
#include "stdlib.h"

#define DT25                  0.025     // 25 ms
#define GPSFILT               0.012422  // Quaternion filter coefficient = 1-exp(-DT25/2s)
#define R2D                   57.295779 // Radians to degrees
#define D2R                   0.0174532 // Degrees to radians
#define MAXSIZE               16

#define PARAM_DEF0( S ) ssGetSFcnParam( S, 0 )

#define MDL_CHECK_PARAMETERS
#if defined( MDL_CHECK_PARAMETERS ) && defined( MATLAB_MEX_FILE )
/*----------------------------------------------------------------------------*/
static void mdlCheckParameters( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
   unsigned short int  i ;
   bool           validParam = false ;


   /* All parameters must be scalar */
   for( i=0 ; i<ssGetSFcnParamsCount( S ) ; i++ )
   {
      const mxArray *pVal = ssGetSFcnParam( S , i ) ;

      if( !mxIsNumeric( pVal ) || !mxIsDouble( pVal ) ||  mxIsLogical( pVal )
      ||   mxIsComplex( pVal ) ||  mxIsSparse( pVal ) 
      ||  !mxIsFinite( mxGetPr( pVal )[0] ) )
      {
         validParam = true ;
         break ;
      }
   }

   if( validParam ) 
   {
      ssSetErrorStatus( S, "All parameters must be a scalar or vectors" ) ;
      return ;
   }

   /* All parameters are not tunable */
   for( i=0 ; i<ssGetSFcnParamsCount( S ) ; i++ )
   {
      ssSetSFcnParamTunable( S, i, 0 ) ;
   }
}
#endif

/*----------------------------------------------------------------------------*/
static void mdlInitializeSizes( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
   ssSetNumSFcnParams( S, NPARAMS ) ;        /* Number of expected parameters */
#if defined( MATLAB_MEX_FILE )
   if( ssGetNumSFcnParams( S ) == ssGetSFcnParamsCount( S ) )
   {
      mdlCheckParameters( S ) ;
      if( ssGetErrorStatus( S ) != NULL ) return ;
   }
   /* Parameter mismatch will be reported by Simulink */
   else  return ;
#endif
    
   ssSetNumContStates( S, NUM_CONT_STATES ) ;
   ssSetNumDiscStates( S, NUM_DISC_STATES ) ;

   if( !ssSetNumInputPorts( S, NUM_INPUTS ) ) return ;

   ssSetInputPortWidth( S, 0, INPUT_0_WIDTH ) ;
   ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH) ;
   ssSetInputPortRequiredContiguous(S, 0, 1);

   ssSetInputPortWidth( S, 1, INPUT_1_WIDTH ) ;
   ssSetInputPortDirectFeedThrough( S, 1, INPUT_1_FEEDTHROUGH ) ;
   ssSetInputPortRequiredContiguous( S, 1, 1 ) ;

   ssSetInputPortWidth( S, 2, INPUT_2_WIDTH ) ;
   ssSetInputPortDirectFeedThrough( S, 2, INPUT_2_FEEDTHROUGH ) ;
   ssSetInputPortRequiredContiguous( S, 2, 1 ) ;

   ssSetInputPortWidth( S, 3, INPUT_3_WIDTH ) ;
   ssSetInputPortDirectFeedThrough( S, 3, INPUT_3_FEEDTHROUGH ) ;
   ssSetInputPortRequiredContiguous( S, 3, 1 ) ;

   if( !ssSetNumOutputPorts( S, NUM_OUTPUTS ) ) return;
   ssSetOutputPortWidth( S, 0, OUTPUT_0_WIDTH ) ;
   ssSetOutputPortWidth( S, 1, OUTPUT_1_WIDTH ) ;
   ssSetOutputPortWidth( S, 2, OUTPUT_2_WIDTH ) ;
   ssSetOutputPortWidth( S, 3, OUTPUT_3_WIDTH ) ;
   ssSetOutputPortWidth( S, 4, OUTPUT_4_WIDTH ) ;
   ssSetOutputPortWidth( S, 5, OUTPUT_5_WIDTH ) ;

   ssSetNumSampleTimes( S, 2 ) ;
   ssSetNumRWork( S, 0 ) ;
   ssSetNumIWork( S, 0 ) ;
   ssSetNumPWork( S, 30 ) ;
   ssSetNumModes( S, 0 ) ;
   ssSetNumNonsampledZCs( S, 0 ) ;

   /* Take care when specifying exception free code - see sfuntmpl_doc.c */
   ssSetOptions( S, SS_OPTION_EXCEPTION_FREE_CODE
                  | SS_OPTION_DISCRETE_VALUED_OUTPUT ) ;
}

/*----------------------------------------------------------------------------*/
static void mdlInitializeSampleTimes( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
   ssSetSampleTime( S, 0, SAMPLE_TIME_0 ) ;
   ssSetOffsetTime( S, 0, 0.0 ) ;
   ssSetSampleTime( S, 1, SAMPLE_TIME_1 ) ;
   ssSetOffsetTime( S, 1, 0.0 ) ;
}

#define MDL_START
#if defined( MDL_START )
/*----------------------------------------------------------------------------*/
static void mdlStart( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   /* ============ */
   double  *X ;
   double  *pqr ;
   double  *A ;
   double  *P ;
   double  *Q ;
   double  *DCM ;
   double  *CPR ;
   double  *YPR ;
   double  *YY ;
   double  *CY ;
   double  *YPR_Acc ;
   double  *YY_GPS ;
   double  *YY_filt ;
   double  *RPR ;
   double  *RY ;
   double  *EPR ;
   double  *EY ;
   double  *KPR ;
   double  *KY ;
   double  *Q_filt ;


   /* Allocate memory to arrays */
   /* ========================= */
   X       = ( double * ) calloc(  7, sizeof( double ) ) ;
   pqr     = ( double * ) calloc(  3, sizeof( double ) ) ;
   A       = ( double * ) calloc( 49, sizeof( double ) ) ;
   P       = ( double * ) calloc( 49, sizeof( double ) ) ;
   Q       = ( double * ) calloc( 49, sizeof( double ) ) ;
   DCM     = ( double * ) calloc(  9, sizeof( double ) ) ;
   CPR     = ( double * ) calloc( 21, sizeof( double ) ) ;
   YPR     = ( double * ) calloc(  3, sizeof( double ) ) ;
   CY      = ( double * ) calloc( 14, sizeof( double ) ) ;
   YY      = ( double * ) calloc(  2, sizeof( double ) ) ;
   YPR_Acc = ( double * ) calloc(  3, sizeof( double ) ) ;
   YY_GPS  = ( double * ) calloc(  2, sizeof( double ) ) ;
   YY_filt = ( double * ) calloc(  2, sizeof( double ) ) ;
   RPR     = ( double * ) calloc(  9, sizeof( double ) ) ;
   RY      = ( double * ) calloc(  4, sizeof( double ) ) ;
   EPR     = ( double * ) calloc(  3, sizeof( double ) ) ;
   EY      = ( double * ) calloc(  2, sizeof( double ) ) ;
   KPR     = ( double * ) calloc( 21, sizeof( double ) ) ;
   KY      = ( double * ) calloc( 14, sizeof( double ) ) ;
   Q_filt  = ( double * ) calloc(  4, sizeof( double ) ) ;


   /* Attach pointers as user data */
   /* ============================ */
   ssGetPWork( S )[ 0] = X ;
   ssGetPWork( S )[ 1] = pqr ;
   ssGetPWork( S )[ 2] = A ;
   ssGetPWork( S )[ 3] = P ;
   ssGetPWork( S )[ 4] = Q ;
   ssGetPWork( S )[ 5] = DCM ;
   ssGetPWork( S )[ 6] = CPR ;
   ssGetPWork( S )[ 7] = YPR ;
   ssGetPWork( S )[ 8] = CY ;
   ssGetPWork( S )[ 9] = YY ;
   ssGetPWork( S )[10] = YPR_Acc ;
   ssGetPWork( S )[11] = YY_GPS ;
   ssGetPWork( S )[12] = YY_filt ;
   ssGetPWork( S )[13] = RPR ;
   ssGetPWork( S )[14] = RY ;
   ssGetPWork( S )[15] = EPR ;
   ssGetPWork( S )[16] = EY ;
   ssGetPWork( S )[17] = KPR ;
   ssGetPWork( S )[18] = KY ;
   ssGetPWork( S )[19] = Q_filt ;
}
#endif

#define MDL_INITIALIZE_CONDITIONS
#if defined( MDL_INITIALIZE_CONDITIONS )
/*----------------------------------------------------------------------------*/
static void mdlInitializeConditions( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   /* ============ */
   real_T  *u0 = ( const real_T * ) ssGetInputPortSignal( S, 0 ) ;
   real_T  *u1 = ( const real_T * ) ssGetInputPortSignal( S, 1 ) ;
   real_T  *u2 = ( const real_T * ) ssGetInputPortSignal( S, 2 ) ;
   real_T  *u3 = ( const real_T * ) ssGetInputPortSignal( S, 3 ) ;

   double  *X      = ssGetPWork( S )[0] ;
   double  *A      = ssGetPWork( S )[2] ;
   double  *P      = ssGetPWork( S )[3] ;
   double  *Q      = ssGetPWork( S )[4] ;
   double  *CPR    = ssGetPWork( S )[6] ;
   double  *CY     = ssGetPWork( S )[8] ;
   double  *RPR    = ssGetPWork( S )[13] ;
   double  *RY     = ssGetPWork( S )[14] ;
   double  *Q_filt = ssGetPWork( S )[19] ;

   double  Yaw, Pitch, Roll ;

   int     i, j ;

   FILE*   deb_file_ptr ;


   /* Open debug file */
   deb_file_ptr = fopen( "deb_file.txt", "a" ) ;


   /* Initial Euler angles */
   /* ==================== */
   Roll  = 0. * D2R ;
   Pitch = 1.3674 * D2R ;
   Yaw   = 0. * D2R ;


   /* Initial Quaternion */
   /* ================== */
   euler2quat( Roll, Pitch, Yaw, X ) ;


   /* Initial gyro offsets */
   /* ==================== */
   for( i=4 ; i<7 ; i++ ) X[i] = 0.0 ;


   /* Initialize Jacobian of F = A matrix */
   /* =================================== */
   for( i=0 ; i<49 ; i++ ) A[i] = 0.0 ;
   A[32] = 1.0 ; A[40] = 1.0 ; A[48] = 1.0 ;


   /* Initialize Jacobian of H for pitch roll = CPR matrix */
   /* ==================================================== */
   for( i=0 ; i<21 ; i++ ) CPR[i] = 0.0 ;


   /* Initialize Jacobian of H for yaw = CY matrix */
   /* ============================================ */
   for( i=0 ; i<14 ; i++ ) CY[i] = 0.0 ;


   /* Initialize state covariance matrix P */
   /* ==================================== */
   for( i=0 ; i<49 ; i++ ) P[i]   = 0.0 ;
   for( i=0 ; i< 4 ; i++ ) P[i*8] = 0.0 ;
   for( i=4 ; i< 7 ; i++ ) P[i*8] = 0.1 ;


   /* Initialize process noise covariance matrix Q */
   /* ============================================ */
   for( i=0 ; i<49 ; i++ ) Q[i]   = 0.0 ;
   for( i=0 ; i< 4 ; i++ ) Q[i*8] = 0.0 ;
   for( i=4 ; i< 7 ; i++ ) Q[i*8] = 0.0 ;


   /* Initialize pitch roll noise covariance matrix RPR */
   /* ================================================= */
   for( i=0 ; i<9 ; i++ ) RPR[i] = 0.0 ;
   RPR[0] = .5 ; RPR[4] = .5 ; RPR[8] = .5 ;


   /* Initialize yaw noise covariance matrix RY */
   /* ========================================= */
   for( i=0 ; i<4 ; i++ ) RY[i] = 0.0 ;
   RY[0] = 5. ; RY[3] = 5. ;


   /* Initialize quaternion state filter */
   /* ================================== */
   for( i=0 ; i<4 ; i++ ) Q_filt[i] = 0.0 ;

/*
   for( i=0 ; i<7 ; i++ )
   {
      for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", P[i*7+j] ) ; 
      fprintf( deb_file_ptr, "\n" ) ;
   }
   fprintf( deb_file_ptr, "\n" ) ;

   for( i=0 ; i<7 ; i++ )
   {
      for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", Q[i*7+j] ) ; 
      fprintf( deb_file_ptr, "\n" ) ;
   }
   fprintf( deb_file_ptr, "\n" ) ;
*/

   /* Close debug file */
   fclose( deb_file_ptr ) ;
}
#endif

#define MDL_OUTPUTS
#if defined( MDL_OUTPUTS )
/*----------------------------------------------------------------------------*/
static void mdlOutputs( SimStruct *S, int_T tid )
/*----------------------------------------------------------------------------*/
{
   /* Declarations */
   /* ============ */
   real_T  *y0 = ssGetOutputPortRealSignal( S, 0 ) ;
   real_T  *y1 = ssGetOutputPortRealSignal( S, 1 ) ;
   real_T  *y2 = ssGetOutputPortRealSignal( S, 2 ) ;
   real_T  *y3 = ssGetOutputPortRealSignal( S, 3 ) ;
   real_T  *y4 = ssGetOutputPortRealSignal( S, 4 ) ;
   real_T  *y5 = ssGetOutputPortRealSignal( S, 5 ) ;

   double  *X   = ssGetPWork( S )[0] ;
   double  *DCM = ssGetPWork( S )[5] ;

   double  Yaw, Pitch, Roll ;


   /* Compute Euler angles from quaternion */
   /* ==================================== */
   quat2euler( X, &Roll, &Pitch, &Yaw ) ;


   /* Outputs */
   /* ======= */
   y0[0] = ( real_T )( Roll ) ;
   y0[1] = ( real_T )( Pitch ) ;
   y0[2] = ( real_T )( Yaw ) ;

   y1[0] = 0.0 ;
   y1[1] = 0.0 ;
   y1[2] = 0.0 ;

   y2[0] = 0.0 ;
   y2[1] = 0.0 ;
   y2[2] = 0.0 ;

   y3[0] = 0.0 ;
   y3[1] = 0.0 ;
   y3[2] = 0.0 ;

   y4[0] = 0.0 ;
   y4[1] = 0.0 ;
   y4[2] = 0.0 ;

   y5[0] = 0.0 ;
   y5[1] = 0.0 ;
   y5[2] = 0.0 ;
}
#endif

#define MDL_UPDATE
#if defined( MDL_UPDATE )
/*----------------------------------------------------------------------------*/
static void mdlUpdate( SimStruct *S, int_T tid )
/*----------------------------------------------------------------------------*/
{
   real_T  *u0 = ( const real_T * ) ssGetInputPortSignal( S, 0 ) ;
   real_T  *u1 = ( const real_T * ) ssGetInputPortSignal( S, 1 ) ;
   real_T  *u2 = ( const real_T * ) ssGetInputPortSignal( S, 2 ) ;
   real_T  *u3 = ( const real_T * ) ssGetInputPortSignal( S, 3 ) ;

   double  *X       = ssGetPWork( S )[ 0] ;
   double  *pqr     = ssGetPWork( S )[ 1] ;
   double  *A       = ssGetPWork( S )[ 2] ;
   double  *P       = ssGetPWork( S )[ 3] ;
   double  *Q       = ssGetPWork( S )[ 4] ;
   double  *CPR     = ssGetPWork( S )[ 6] ;
   double  *YPR     = ssGetPWork( S )[ 7] ;
   double  *CY      = ssGetPWork( S )[ 8] ;
   double  *YY      = ssGetPWork( S )[ 9] ;
   double  *YPR_Acc = ssGetPWork( S )[10] ;
   double  *YY_GPS  = ssGetPWork( S )[11] ;
   double  *YY_filt = ssGetPWork( S )[12] ;
   double  *RPR     = ssGetPWork( S )[13] ;
   double  *RY      = ssGetPWork( S )[14] ;
   double  *EPR     = ssGetPWork( S )[15] ;
   double  *EY      = ssGetPWork( S )[16] ;
   double  *KPR     = ssGetPWork( S )[17] ;
   double  *KY      = ssGetPWork( S )[18] ;
   double  *Q_filt  = ssGetPWork( S )[19] ;

   double  V_buff[3] ;
   double  d_buff1, d_buff2 ;

   int      i, j ;

   FILE    *deb_file_ptr ;


   /* Open debug file */
   deb_file_ptr = fopen( "deb_file.txt", "a" ) ;

/*============================================================================*/
/*============================================================================*/
/*                                25ms routine                                */
/*============================================================================*/
/*============================================================================*/

   /* If 25ms sample time */
   if( ssIsSampleHit( S, 0, tid ) )
   {
      /* Prediction stage */
      /* ================ */
      /* Compute body angular velocity: correct gyrometers for offsets */
      for( i=0; i<3; i++ ) pqr[i] = u0[i] - X[4+i] ;

      /* Assemble state transition matrix */
      assemble_A_matrix( X, pqr, A, ( ( ( double )( DT25 ) )/2.0 ) ) ;

      /* Propagate state */
      propagate_state( X, A ) ;

      /* Propagate state covariance */
      propagate_covariance( P, A, Q, ( ( double )( DT25 ) ) ) ;


      /* Quaternion phasing */
      /* ================== */
      /* Filter the quaternion with a 1st order */ 
      for( i=0 ; i<4 ; i++ ) Q_filt[i] += GPSFILT * ( X[i] - Q_filt[i] ) ;
   
/*
      fprintf( deb_file_ptr, "\n" );
      for( i=0 ; i<7 ; i++ ) fprintf( deb_file_ptr, " %9.6f ", X[i] ) ;
      fprintf( deb_file_ptr, "\n" ) ;
      fprintf( deb_file_ptr, "\n" ) ;
      for( i=0 ; i<7 ; i++ )
      {
         for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", A[i*7+j] ) ; 
         fprintf( deb_file_ptr, "\n" ) ;
      }

      fprintf( deb_file_ptr, "\n" ) ;
      for( i=0 ; i<7 ; i++ )
      {
         for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", P[i*7+j] ) ; 
         fprintf( deb_file_ptr, "\n" ) ;
      }
*/

   }

/*============================================================================*/
/*============================================================================*/
/*                                1s routine                                  */
/*============================================================================*/
/*============================================================================*/

   /* If 1s sample time */
   if( ssIsSampleHit( S, 1, tid ) )
   {
      /* Pitch Roll correction */
      /* ===================== */
      /* Read body acceleration (x y z accelerometers) */
      for ( i=0 ; i<3 ; i++ ) V_buff[i] = u1[i] ;

      /* Compute magnitude of speed over ground from GPS */
      d_buff1 = sqrt( u2[0]*u2[0] + u2[1]*u2[1] ) ;

      /* Correct body acceleration for centrifugal effect */
      V_buff[1] += pqr[2] * d_buff1 ;      
      V_buff[2] -= pqr[1] * d_buff1 ;      

      /* Normalize body acceleration to yield the measured z earth vector */
      VectorDotProduct( V_buff, V_buff, &d_buff1 ) ;
      d_buff1 = 1.0 / sqrt( d_buff1 ) ;
      VectorScale( V_buff, &d_buff1, YPR_Acc ) ;

      /* Compute estimated z earth vector */
      compute_YPR_vector( X, YPR ) ;

      /* Assemble z earth vector observation matix */
      assemble_CPR_matrix( X, CPR ) ;

      /* Compute pitch roll error */
      EPR[0] = YPR_Acc[0] - YPR[0] ;
      EPR[1] = YPR_Acc[1] - YPR[1] ;
      EPR[2] = YPR_Acc[2] - YPR[2] ;

      /* Update state */
      kalmanUpdate( P, X, CPR, RPR, EPR, KPR, 7, 3 ) ;


      /* Yaw correction */
      /* ============== */
      /* Compute estimated filtered x body over ground */
      compute_YY_vector( Q_filt, YY_filt ) ;

      d_buff1 = sqrt( YY_filt[0]*YY_filt[0] + YY_filt[1]*YY_filt[1] ) ;

      /* Scale GPS speed over ground with x body over ground magnitude */
      YY_GPS[0] = cos( u3[0] ) * d_buff1 ;
      YY_GPS[1] = sin( u3[0] ) * d_buff1 ;

      /* Assemble filtered x body over ground observation matix */
      assemble_CY_matrix( Q_filt, CY ) ;

      /* Compute filtered yaw error */
      EY[0] = YY_GPS[0] - YY_filt[0] ;
      EY[1] = YY_GPS[1] - YY_filt[1] ;

      /* Update state */
      kalmanUpdate( P, X, CY, RY, EY, KY, 7, 2 ) ;

/*
      fprintf( deb_file_ptr, "\n" ) ;
      for( i=0 ; i<3 ; i++ )
      {
         for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", KPR[i*7+j] ) ; 
         fprintf( deb_file_ptr, "\n" ) ;
      }

      fprintf( deb_file_ptr, "\n" ) ;
      for( i=0 ; i<2 ; i++ )
      {
         for( j=0 ; j<7 ; j++ ) fprintf( deb_file_ptr, " %9.6f ", KY[i*7+j] ) ; 
         fprintf( deb_file_ptr, "\n" ) ;
      }
*/
   }

   /* Close debug file */
   fclose( deb_file_ptr ) ;
}
#endif


#undef MDL_DERIVATIVES
#if defined( MDL_DERIVATIVES )
/*----------------------------------------------------------------------------*/
static void mdlDerivatives( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
}
#endif

#if defined( MATLAB_MEX_FILE ) || defined( NRT )
#undef MDL_ZERO_CROSSINGS
  #if defined( MDL_DERIVATIVES )
/*----------------------------------------------------------------------------*/
static void mdlZeroCrossings( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
}
  #endif
#endif

/*----------------------------------------------------------------------------*/
static void mdlTerminate( SimStruct *S )
/*----------------------------------------------------------------------------*/
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
  #include "simulink.c"    /* MEX-file interface mechanism */
#else
  #include "cg_sfun.h"     /* Code generation registration function */
#endif


//============================================================================//
//                        MATH LIBRARY from RotoMotion                        //
//============================================================================//
/*----------------------------------------------------------------------------*/
/* This will add vector a with vector b and return vector c
/* c(n,1) + a(n,1) + b(n,1)
/*----------------------------------------------------------------------------*/
void VVadd( double *a, double *b, double *c, int n )
{
   int  i ;


   for( i=0 ; i<n ; ++i ) c[i] = a[i] + b[i] ;
}

/*----------------------------------------------------------------------------*/
/* This will zero out the matrix A
/* A = zeros (n,m)
/*----------------------------------------------------------------------------*/
void Minit( double *A, int n, int m )
{
   int  i, j ;


   for(i=0 ; i<n ; ++i ) for( j=0 ; j<m ; ++j ) A[i*m+j] = 0.0 ;
}

/*----------------------------------------------------------------------------*/
/* This will generate an identity matrix I
/* A(n,n) = eye (n)
/*----------------------------------------------------------------------------*/
void eye ( double *I, int n )
{
   int  i ;


   Minit( I, n, n ) ; for( i=0 ; i<n ; ++i ) I[i*n+i] = 1.0 ;
}

/*----------------------------------------------------------------------------*/
/* This will multiply scalar value s will all elements of matrix A(m,n)
/* placing result into matrix B(m,n)
/* B(m,n) = s. * A(m,n)
/*----------------------------------------------------------------------------*/
void sMmult( double s, double *A, double *B, int m, int n )
{
   int  i, j ;


   for( i=0 ; i<m ; ++i ) for ( j=0 ; j<n ; ++j ) B[i*n+j] = s * A[i*n+j] ;
}

/*----------------------------------------------------------------------------*/
/* This will multiply matrix A and matrix B return in matrix C
/* C(m,p) = A(m,n) * B(n,p)
/*----------------------------------------------------------------------------*/
void MMmult( double *A, double *B, double *C, int m, int n, int p )
{
   double  s ;
   int     i, j, k ;


   for( i=0 ; i<m ; ++i )
   {
      for( j=0 ; j<p ; ++j )
      {
         s = 0.0 ;

         for( k=0 ; k<n ; ++k ) s += A[i*n+k] * B[k*p+j] ;

         C[i*p+j] = s ;
      }
   }
}

/*----------------------------------------------------------------------------*/
/* This will multiply matrix A and vector b return in vector c
/* c(m,1) = A(m,n) * b(n,1) 
/*----------------------------------------------------------------------------*/
void MVmult( double *A, double *b, double *c, int m, int n )
{
   double  s ;
   int     i, j ;


   for( j=0; j<m; ++j )
   {
      s = 0.0 ;

      for( i=0 ; i<n ; ++i ) s += A[j*n+i] * b[i] ;

      c[j] = s ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will multiply vector a and matrix B return in vector c
/* c(1,n) = a(1,m) * B(m,n)
/*----------------------------------------------------------------------------*/
void VMmult( double *a, double *B, double *c, int m, int n )
{
   double  s ;
   int     i, j ;


   for( i=0 ; i<n ; ++i )
   {
      s = 0.0 ;

      for( j=0 ; j<m ; ++j ) s += a[j] * B[j*n+i] ;

      c[i] = s ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will transpose a matrix/vector A return in matrix/vector B  //checked//
/* B(n,m) = transpose A(m,n)
/*----------------------------------------------------------------------------*/
void transpose( double *A, double *B, int m, int n )
{
   int  i, j, k, l ;

   k = 0 ; // initialize B index

   /* For each row of B */
   for( i=0 ; i<n ; ++i )
   {
      l = i ; // initialize A index to upper element of colum i

      /* For each colum of B */
      for ( j=0 ; j<m ; ++j )
      {
         B[k] = A[l] ;

         k += 1 ; // increment B index
         l += n ; // increment A row #
      }
   }
}

/*----------------------------------------------------------------------------*/
/* This will add matrix A with matrix B and return matrix C //checked//
/* C(n,m) = A(n,m) + B(n,m)
/*----------------------------------------------------------------------------*/
void MMadd( double *A, double *B, double *C, int m, int n )
{
   int  i, j, k;

   k = 0 ;
   for( i=0 ; i<n ; ++i )
     for( j=0 ; j<m ; ++j ) {C[k] = A[k] + B[k] ; k += 1 ;}
}

/*----------------------------------------------------------------------------*/
/* This will subtract matrix A from matrix B and return matrix C //checked//
/* C(n,m) = A(n,m) - B(n,m)
/*----------------------------------------------------------------------------*/
void MMsub(double *A, double *B, double *C, int m, int n )
{
   int  i, j, k;

   k = 0 ;
   for( i=0 ; i<n ; ++i )
     for( j=0 ; j<m ; ++j ) {C[k] = A[k] - B[k] ; k += 1 ;}
}

/*----------------------------------------------------------------------------*/
/* This will perform LU decomp on matrix A return matrix L and matrix U
/* LU(A(n,n)) => L(n,n) and U(n,n)
/*----------------------------------------------------------------------------*/
void LU( double *A, double *L, double *U, int n )
{
   double  Acopy [MAXSIZE*MAXSIZE] ;
   int     i, j, k ;


   /* Copy A matrix */
   for( i=0 ; i<n ; ++i ) for(j=0; j<n; ++j) Acopy[i*n+j] = A[i*n+j] ;


   /* Decompose */
   for( k=0 ; k<n-1 ; ++k )
   {
      for( i=k+1 ; i<n ; ++i )
      {
         Acopy[i*n+k] = Acopy[i*n+k] / Acopy[k*n+k] ;

         for( j=k+1 ; j<n ; ++j ) Acopy[i*n+j] -= Acopy[i*n+k] * Acopy[k*n+j] ;
      }
   }


   /* Extract the L matrix */
   eye( L, n ) ;
   for( j=0 ; j<n-1 ; ++j )
     for ( i=j+1 ; i<n ; ++i ) L[i*n+j] = Acopy[i*n+j] ;


   /* Extract the U matrix */
   Minit( U, n, n ) ;
   for( i=0; i<n; ++i ) for ( j=i; j<n; ++j ) U[i*n+j] = Acopy[i*n+j] ;
}

/*----------------------------------------------------------------------------*/
/* This will take column c from matrix A(m,n) place it into vector a(m)
/*----------------------------------------------------------------------------*/
void Mcol( double *A, double *a, int c, int m, int n )
{
   int  i ;


   for( i=0; i<m; ++i ) a[i] = A[i*n+c] ;
}

/*----------------------------------------------------------------------------*/
/* This will take vector a(m) and place it into column c of matrix A(m,n)
/*----------------------------------------------------------------------------*/
void Vcol( double *a, double *A, int c, int m, int n )
{
   int  i ;


   for( i=0; i<m; ++i ) A[i*n+c] = a[i] ;
}

/*----------------------------------------------------------------------------*/
/* This will solve A*x = b, where matrix A is upper triangular
/* A(n,n)*x(n,1) = b(n,1)
/*----------------------------------------------------------------------------*/
void solveupper( double *A, double *b, double *x, int n )
{
   int  i, j, p ;


   p = n + 1 ;

   for( i=1; i<=n; ++i )
   {
      x[p-i-1] = b[p-i-1] ;

      for( j=(p+1-i); j<=n; ++j ) x[p-i-1] -= A[(p-i-1)*n+(j-1)]*x[j-1] ;

      x[p-i-1] = x[p-i-1] / A[(p-i-1)*n+(p-i-1)] ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will solve A*x = b, where matrix A is lower triangular
/* A(n,n)*x(n,1) = b(n,1)
/*----------------------------------------------------------------------------*/
void solvelower( double *A, double *b, double *x, int n )
{
   int  i, j ;


   for( i=1; i<=n; ++i )
   {
      x[i-1] = b[i-1] ;

      for(j=1; j<=i-1; ++j ) x[i-1] = x[i-1] - A[(i-1)*n+(j-1)]*x[j-1] ;

      x[i-1] = x[i-1]/A[(i-1)*n+(i-1)] ;
   }
}

/*----------------------------------------------------------------------------*/
/* This will perform the inverse on matrix A return in matrix B
/* inv(A(n,n)) = B(n,n)
/*----------------------------------------------------------------------------*/
void inv( double *A, double *B, int n )
{
   double  identCol [MAXSIZE] ;
   double  ident    [MAXSIZE*MAXSIZE] ;
   double  L        [MAXSIZE*MAXSIZE] ;
   double  U        [MAXSIZE*MAXSIZE] ;
   double  invUcol  [MAXSIZE] ;
   double  invLcol  [MAXSIZE] ;
   double  invU     [MAXSIZE*MAXSIZE] ;
   double  invL     [MAXSIZE*MAXSIZE] ;

   double  detA ;
   int     i ;


   /* Case n = 1 */
   /* ---------- */
   if( n == 1 )
   {
      B[0] = 1.0 / A[0] ;

      return ;
   }


   /* Case n = 2 */
   /* ---------- */
   if( n == 2 )
   {
      detA = A[0]*A[3] - A[1]*A[2] ;

      B[0] =  A[3] / detA ;
      B[1] = -A[1] / detA ;
      B[2] = -A[2] / detA ;
      B[3] =  A[0] / detA ;

      return ;
   }

   /* General case */
   /* ------------ */
   /* Perform LU decomposition on A */
   eye( ident, n ) ; LU( A, L, U, n ) ;

   for (i=0; i<n; ++i)
   {
      /* Separates the ith column */
      Mcol( ident, identCol, i, n, n ) ;

      solveupper( U, identCol, invUcol, n ) ;
      solvelower( L, identCol, invLcol, n ) ;

      /* Place invUcol in ith column of invU */
      Vcol( invUcol, invU, i, n, n ) ;

      /* Place invLcol in ith column of invL */
      Vcol( invLcol, invL, i, n, n ) ;
   }

   /* inv(A) = inv(U)*inv(L) */
   MMmult( invU, invL, B, n, n, n ) ;
}

/*----------------------------------------------------------------------------*/
/* This will perform a Kalman filter gain, state, and covariance
/* matrix update.
/*
/*	P(n,n)		Covariance matrix
/*	X(n,1)		State Vector
/*	C(m,n)		Measurement matrix; m=# of measurements, n=# of states
/*	R(m,m)		Measurement weight matrix 
/*	E(m,1)		Error vector = Xmeasurement(m,1) - Xestimate(m,1)
/*	K(n,m)		Kalman Gain matrix
/*----------------------------------------------------------------------------*/
void kalmanUpdate( double *P, double *X, double *C, double *R, double *E,
                   double *K, int n, int m )
{
   double  F   [MAXSIZE*MAXSIZE] ;
   double  T1  [MAXSIZE*MAXSIZE] ;
   double  T2  [MAXSIZE*MAXSIZE] ;
   double  TV1 [MAXSIZE] ;


   /* Perform F = C*P*C' + R */
   /* ---------------------- */
   MMmult( C, P, T1, m, n, n ) ;
   transpose( C, T2, m, n ) ;
   MMmult( T1, T2, F, m, n, m ) ;
   MMadd( F, R, F, m, m ) ;


   /* Perform K = P*C'*inv(F) */ 
   /* ----------------------- */
   MMmult( P, T2, T1, n, n, m ) ;
   inv( F, T2, m ) ;
   MMmult( T1, T2, K, n, m, m ) ;
	

   /* Perform x = x + K*(ys - yp) */
   /* --------------------------- */
   MVmult( K, E, TV1, n, m ) ;
   VVadd( X, TV1, X, n ) ;


   /* Perform P = P - K*C*P */ 
   /* --------------------- */
   MMmult( K, C, T1, n, m, n ) ;
   MMmult( T1, P, T2, n, n, n ) ;
   MMsub( P, T2, P, n, n ) ;
}

/*----------------------------------------------------------------------------*/
/* Euler angles to quaternion vector
/*----------------------------------------------------------------------------*/
void euler2quat( double phi, double theta, double psi, double *Q )
{
   double  phi_2, theta_2, psi_2 ;
   double  sphi, cphi, stheta, ctheta, spsi, cpsi ;
   double  cphi_ctheta , sphi_stheta , cphi_stheta , sphi_ctheta ;


   phi_2 = phi / 2.0 ; theta_2 = theta / 2.0 ; psi_2 = psi / 2.0 ;

   sphi   = sin( phi_2 ) ;    cphi   = cos( phi_2 ) ;
   stheta = sin( theta_2 ) ;  ctheta = cos( theta_2 ) ;
   spsi   = sin( psi_2 ) ;    cpsi   = cos( psi_2 ) ;

   cphi_ctheta = cphi * ctheta ; sphi_stheta = sphi * stheta ;
   cphi_stheta = cphi * stheta ; sphi_ctheta = sphi * ctheta ;

   Q[0] =  cphi_ctheta * cpsi + sphi_stheta * spsi ;
   Q[1] = -cphi_stheta * spsi + sphi_ctheta * cpsi ;
   Q[2] =  cphi_stheta * cpsi + sphi_ctheta * spsi ;
   Q[3] =  cphi_ctheta * spsi - sphi_stheta * cpsi ;
}


/*----------------------------------------------------------------------------*/
/* Quaternion vector to euler angles
/*----------------------------------------------------------------------------*/
void quat2euler( double *Q, double *phi, double *theta, double *psi )
{
   *phi   = atan2( 2.0 * ( Q[2] * Q[3] + Q[0] * Q[1] ),
             1.0 - 2.0 * ( Q[1] * Q[1] + Q[2] * Q[2] ) ) ;

   *theta = -asin( 2.0 * ( Q[1] * Q[3] - Q[0] * Q[2] ) ) ;

   *psi   = atan2( 2.0 * ( Q[1] * Q[2] + Q[0] * Q[3] ),
             1.0 - 2.0 * ( Q[2] * Q[2] + Q[3] * Q[3] ) ) ;
}


/*----------------------------------------------------------------------------*/
/* Quaternion vector to Direct Cosine Matrix
/*----------------------------------------------------------------------------*/
void quat2DCM( double *Q, double *R )
{
   R[0] = 1.0 - 2.0 * (Q[2] * Q[2] + Q[3] * Q[3]) ;
   R[3] =       2.0 * (Q[1] * Q[2] + Q[0] * Q[3]) ;
   R[6] =       2.0 * (Q[1] * Q[3] - Q[0] * Q[2]) ;

   R[1] =       2.0 * (Q[1] * Q[2] - Q[0] * Q[3]) ;
   R[4] = 1.0 - 2.0 * (Q[1] * Q[1] + Q[3] * Q[3]) ;
   R[7] =       2.0 * (Q[2] * Q[3] + Q[0] * Q[1]) ;

   R[2] =       2.0 * (Q[1] * Q[3] + Q[0] * Q[2]) ;
   R[5] =       2.0 * (Q[2] * Q[3] - Q[0] * Q[1]) ;
   R[8] = 1.0 - 2.0 * (Q[1] * Q[1] + Q[2] * Q[2]) ;
}

/*----------------------------------------------------------------------------*/
/* Propagate quaternion
/*----------------------------------------------------------------------------*/
void propagate_state( double *Q, double *A )
{
   double  mag ;
   double  q[4] ;
   int     i ;

   for( i=0 ; i<4 ; i++) q[i] = Q[i] ;

   Q[0] = ( A[ 0] * q[0] + A[ 1] * q[1] + A[ 2] * q[2] + A[ 3] * q[3] ) ; 
   Q[1] = ( A[ 7] * q[0] + A[ 8] * q[1] + A[ 9] * q[2] + A[10] * q[3] ) ;
   Q[2] = ( A[14] * q[0] + A[15] * q[1] + A[16] * q[2] + A[17] * q[3] ) ;
   Q[3] = ( A[21] * q[0] + A[22] * q[1] + A[23] * q[2] + A[24] * q[3] ) ;

   mag = 0.0 ; for( i=0 ; i<4 ; i++ ) mag += Q[i]*Q[i] ; mag = sqrt (mag) ;
   for( i=0 ; i<4 ; i++ ) Q[i] /= mag ;
}

/*----------------------------------------------------------------------------*/
/* Assemble A matrix
/*----------------------------------------------------------------------------*/
void assemble_A_matrix( double *Q, double *pqr, double *A, double dt_sur_2 )
{
   double  p_2, q_2, r_2 ;
   double  q02, q12, q22, q32 ;


   p_2 = pqr[0] * dt_sur_2 ;
   q_2 = pqr[1] * dt_sur_2 ;
   r_2 = pqr[2] * dt_sur_2 ;

   q02 = Q[0] * dt_sur_2 ;
   q12 = Q[1] * dt_sur_2 ;
   q22 = Q[2] * dt_sur_2 ;
   q32 = Q[3] * dt_sur_2 ;

   A[ 0] =  1.0 ;
   A[ 1] = -p_2 ;
   A[ 2] = -q_2 ;
   A[ 3] = -r_2 ;
   A[ 4] =  q12 ;
   A[ 5] =  q22 ;
   A[ 6] =  q32 ;

   A[ 7] =  p_2 ;
   A[ 8] =  1.0 ;
   A[ 9] =  r_2 ;
   A[10] = -q_2 ;
   A[11] = -q02 ;
   A[12] =  q32 ;
   A[13] = -q22 ;

   A[14] =  q_2 ;
   A[15] = -r_2 ;
   A[16] =  1.0 ;
   A[17] =  p_2 ;
   A[18] = -q32 ;
   A[19] = -q02 ;
   A[20] =  q12 ;

   A[21] =  r_2 ;
   A[22] =  q_2 ;
   A[23] = -p_2 ;
   A[24] =  1.0 ;
   A[25] =  q22 ;
   A[26] = -q12 ;
   A[27] = -q02 ;

   // Terms for offsets are equal to 1 and are set during initialization
}

/*----------------------------------------------------------------------------*/
/* Propagate state covariance
/*----------------------------------------------------------------------------*/
void propagate_covariance( double *P, double *A , double *Q )
{
   double  tA[49] ;
   double  AP[49] ;
   double  APtA[49] ;


   transpose( A, tA, 7, 7 ) ;			// tA

   MMmult( A, P, AP, 7, 7, 7 ) ;		// A.P

   MMmult( AP, tA, APtA, 7, 7, 7 ) ;		// A.P.tA

   MMadd( APtA, Q, P, 7 , 7 ) ;			// A.P.tA + Q
}

/*----------------------------------------------------------------------------*/
/* Assemble pitch roll output CPR matrix
/*----------------------------------------------------------------------------*/
void assemble_CPR_matrix( double *Q, double *CPR )
{
   CPR[ 0] = -Q[2] * 2.0 ;
   CPR[ 1] =  Q[3] * 2.0 ;
   CPR[ 2] = -Q[0] * 2.0 ;
   CPR[ 3] =  Q[1] * 2.0 ;

   CPR[ 7] =  Q[1] * 2.0 ;
   CPR[ 8] =  Q[0] * 2.0 ;
   CPR[ 9] =  Q[3] * 2.0 ;
   CPR[10] =  Q[2] * 2.0 ;

   CPR[14] =  0.0;
   CPR[15] = -Q[1] * 4.0 ;
   CPR[16] = -Q[2] * 4.0 ;
   CPR[17] =  0.0 ;
}

/*----------------------------------------------------------------------------*/
/* Assemble yaw output CY matrix
/*----------------------------------------------------------------------------*/
void assemble_CY_matrix( double *Q, double *CY )
{
// CY[ 0] =  0.0 ;
// CY[ 1] =  0.0 ;
   CY[ 2] = -Q[2] * 4.0 ;
   CY[ 3] = -Q[3] * 4.0 ;

   CY[ 7] =  Q[3] * 2.0 ;
   CY[ 8] =  Q[2] * 2.0 ;
   CY[ 9] =  Q[1] * 2.0 ;
   CY[10] =  Q[0] * 2.0 ;
}

/*----------------------------------------------------------------------------*/
/* Compute YPR vector
/*----------------------------------------------------------------------------*/
void compute_YPR_vector( double *Q, double *YPR )
{
   YPR[0] =       2.0 * (Q[1] * Q[3] - Q[0] * Q[2]) ;
   YPR[1] =       2.0 * (Q[2] * Q[3] + Q[0] * Q[1]) ;
   YPR[2] = 1.0 - 2.0 * (Q[1] * Q[1] + Q[2] * Q[2]) ;
}

/*----------------------------------------------------------------------------*/
/* Compute YY vector
/*----------------------------------------------------------------------------*/
void compute_YY_vector( double *Q, double *YY )
{
   YY[0] = 1.0 - 2.0 * (Q[2] * Q[2] + Q[3] * Q[3]) ;
   YY[1] =       2.0 * (Q[1] * Q[2] + Q[0] * Q[3]) ;
}

/*----------------------------------------------------------------------------*/
/* Scalar Product = V1.V2                                                     */
/*----------------------------------------------------------------------------*/
void VectorDotProduct( double *V1, double *V2, double *Scalar_Product )
{
   int    i;


   *Scalar_Product = 0.0 ;
   for (i=0;i<3;i++) *Scalar_Product += V1[i] * V2[i] ;
}

/*----------------------------------------------------------------------------*/
/* V3 = V1+V2                                                                 */
/*----------------------------------------------------------------------------*/
void VectorScale( double *V1, double *K, double *V2 )
{
   int    i;


   for (i=0;i<3;i++) V2[i] = V1[i] * (*K);
}