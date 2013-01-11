//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief   Log manager header file
///
/// \file
///
//  Change   Log task moved to Log.c
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

/// log message structure
typedef struct
{
  uint8_t ucLength;   //!< length of message
  uint16_t *pcData;   //!< pointer to message content
} xLog_Message;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/// queue of log messages
VAR_GLOBAL xQueueHandle xLog_Queue;

/*---------------------------------- Interface -------------------------------*/

void Log_Task( void *pvParameters );
