//============================================================================
//
// $RCSfile: Telemetry.h,v $ (HEADER FILE)
// $Revision: 1.1 $
// $Date: 2011/01/22 16:49:58 $
// $Author: Lorenz $
//
/// \brief Simulation interface
/// \file
///
//  CHANGES added telemetry task
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

typedef struct
{
  uint8_t ucLength;
  uint16_t *pcData;
} xTelemetry_Message;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*---------------------------------- Interface -------------------------------*/

void Telemetry_Init( void ) ;
void Telemetry_Task( void *pvParameters );
bool Telemetry_Parse ( void );
void Telemetry_Send_Controls ( void );
void Telemetry_Send_Waypoint ( void );
bool Sim_Settled ( void ) ;
float Sim_Speed ( void );
float Sim_GetData ( int n );
void Sim_SetData ( int iIndex, float fVal );
void Sim_SaveOffsets ( void );


