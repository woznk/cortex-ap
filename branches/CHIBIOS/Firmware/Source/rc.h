/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Remote control PPM input
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#define PPM_SIGNAL_OK       6   /* radio signal is OK */
#define PPM_NO_SIGNAL       0   /* radio signal is bad */

/* Control mode definitions */
#define MODE_UNDEFINED      0   /* mode is undefined */
#define MODE_MANUAL         1   /* manual control */
#define MODE_STAB           2   /* stabilize roll and pitch */
#define MODE_NAV            3   /* navigate */
#define MODE_RTL            4   /* return to launch */
#define MODE_FPV            5   /* stabilize camera for FPV */
#define MODE_NUM            6

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void Init_RC ( void );
int16_t Get_RC_Channel ( uint8_t ucChannel );
uint8_t Get_RC_Mode ( void );
uint8_t Get_RC_Signal ( void );
