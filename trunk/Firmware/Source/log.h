//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief   Log manager header file
//  CHANGES  Log task moved to Log.c
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
} xLog_Message;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

VAR_GLOBAL xQueueHandle xLog_Queue;

/*---------------------------------- Interface -------------------------------*/

void Log_Init ( void );
void Log_DCM ( void );
void Log_PPM ( void );
void Log_PutChar( char c );
void Log_Send(uint16_t *data, uint8_t num);
void Log_Task( void *pvParameters );
