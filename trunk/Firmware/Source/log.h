//============================================================================
//
// $RCSfile: log.h,v $ (HEADER FILE)
// $Revision: 1.3 $
// $Date: 2010/04/14 17:36:37 $
// $Author: Lorenz $
//
//  LANGUAGE C
/// \brief   Log manager header file
//  CHANGES  Log_Send() argumento changed from int to uint16_t
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

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Log_Init ( void );
void Log_DCM ( void );
void Log_PPM ( void );
void Log_PutChar( char c );
void Log_Send(uint16_t data);
