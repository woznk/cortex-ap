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
//  Change: (Lint) added expliicit cast to type to all #definitions 
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

#define PPM_SIGNAL_OK       (uint8_t)6   ///< radio signal is OK
#define PPM_NO_SIGNAL       (uint8_t)0   ///< radio signal is bad

/* Control mode definitions */
#define MODE_UNDEFINED      (uint8_t)0   ///< mode is undefined
#define MODE_STAB           (uint8_t)1   ///< stabilize roll and pitch
#define MODE_NAV            (uint8_t)2   ///< navigate
#define MODE_MANUAL         (uint8_t)3   ///< manual control
#define MODE_RTL            (uint8_t)4   ///< return to launch
#define MODE_FPV            (uint8_t)5   ///< stabilize camera for FPV

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

