//============================================================================+
//
// $RCSfile: $ (SOURCE FILE)
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
/// PPM input driver header file
///
//  CHANGES #defined MODE_CHANNEL and AILERON_CHANNEL
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
#define RC_CHANNELS     7   ///< Number of RC channels. Modify according to RC type
#define MODE_CHANNEL    4   // Mode selection channel
#define AILERON_CHANNEL 1   // Aileron control channel


/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PPM_Init( void );
unsigned long PPMGetChannel( unsigned char ucChannel );
unsigned char PPMSignalStatus( void );

