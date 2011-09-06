//============================================================================
//
// $RCSfile: Telemetry.h,v $ (HEADER FILE)
// $Revision: 1.1 $
// $Date: 2011/01/22 16:49:58 $
// $Author: Lorenz $
//
/// \brief Simulation interface
///
/// \file
///
//  CHANGES eliminato SIMULATION (sostituito da SIMULATOR, dove serve)
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

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

tBoolean Telemetry_Parse ( void );
void Telemetry_Send_Controls ( void );
void Telemetry_Send_Waypoint ( void );
tBoolean Sim_Settled ( void ) ;
float Sim_Speed ( void );
float Sim_GetData ( int n );
void Sim_SetData ( int iIndex, float fVal );
void Sim_SaveOffsets ( void );


