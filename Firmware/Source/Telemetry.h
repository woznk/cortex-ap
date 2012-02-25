//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Simulation interface
/// \file
///
//  CHANGES unused interface functions made static
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

typedef struct {
  uint8_t ucLength;
  uint16_t *pcData;
} xTelemetry_Message;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*---------------------------------- Interface -------------------------------*/

void Telemetry_Task( void *pvParameters );
bool Sim_Settled ( void ) ;
float Sim_Speed ( void );
float Sim_GetData ( int n );
void Sim_SetData ( int iIndex, float fVal );
void Sim_SaveOffsets ( void );


