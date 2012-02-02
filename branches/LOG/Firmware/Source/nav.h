//============================================================================
//
// $RCSfile: nav.h,v $ (HEADER FILE)
// $Revision: 1.2 $
// $Date: 2010/01/25 21:42:02 $
// $Author: Lorenz $
//
/// \file
/// \brief  Navigation manager header file
//  CHANGES Navigate() function changed into navigation task 
//          added type definition of gps message
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

typedef struct
{
  uint8_t ucLength;
  uint16_t *pcData;
} xGps_Message;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

VAR_GLOBAL xQueueHandle xGps_Queue;

/*---------------------------------- Interface -------------------------------*/

void Navigation_Task( void *pvParameters );
int16_t Nav_Bearing ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_WaypointIndex ( void );

