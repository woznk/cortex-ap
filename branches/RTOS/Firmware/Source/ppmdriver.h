//============================================================================+
//
// $RCSfile: ppmdriver.h,v $ (SOURCE FILE)
// $Revision: 1.2 $
// $Date: 2010/04/14 17:30:52 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             PPM input driver header file
//
//  CHANGES     spostato qui #define numero canali RC
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define PPM_SIGNAL_OK   1
#define PPM_SIGNAL_BAD  0
#define RC_CHANNELS     6   ///< Number of RC channels. Modify according to RC type

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PPMInit( void );
unsigned long PPMGetChannel( unsigned char ucChannel );
unsigned char PPMSignalStatus( void );

