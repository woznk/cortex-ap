//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief PPM input driver header file
///
/// \file
///
//  Change  #definition of radio channels model moved to config.h
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

#define PPM_SIGNAL_OK       6   ///< radio signal is OK
#define PPM_NO_SIGNAL       0   ///< radio signal is bad

/* Control mode definitions */
#define MODE_UNDEFINED      0   ///< mode is undefined
#define MODE_STAB           1   ///< stabilize roll and pitch
#define MODE_NAV            2   ///< navigate
#define MODE_MANUAL         3   ///< manual control
#define MODE_RTL            4   ///< return to launch
#define MODE_FPV            5   ///< stabilize camera for FPV

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PPM_Init(void);
uint8_t PPMSignalStatus(void);
uint8_t PPMGetMode(void);
int16_t PPMGetChannel(uint8_t ucChannel);

