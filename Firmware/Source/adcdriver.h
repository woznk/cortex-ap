//============================================================================
//
// $RCSfile: adcdriver.h,v $ (HEADER FILE)
// $Revision: 1.5 $
// $Date: 2010/12/30 09:44:03 $
// $Author: Lorenz $
//
/// \brief ADC driver header file
///
/// \file
/// NB: la frequenza di aggiornamento della matrice DCM può essere di 50 Hz
/// quando i calcoli sono fatti dal Cortex, non può essere maggiore di 15 Hz
/// se i calcoli sono fatti dal PC. Eventualmente misurare il tempo tra due
/// chiamate successive di DCMupdate().
///
/// \todo Spostare SAMPLES_PER_SECOND e DELTA_T in un file Config.h o simile
///
/// CHANGES Spostate #definizioni nel file config.h
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

void ADCInit (void);
int ADCSettled (void);
unsigned char ADCSamples (void);
float ADCGetData (int n);
