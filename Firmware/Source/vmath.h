//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
/// Vector algebra header file
//
//  CHANGES
//
//
//
//============================================================================*/

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

float VectorDotProduct(float fVectorA[3], float fVectorB[3]);
void VectorCrossProduct(float fCrossP[3], float fVectorA[3], float fVectorB[3]);
void VectorScale(float fScaledV[3],float fVector[3], float fScale);
void VectorAdd(float fSumV[3], float fVectorA[3], float fVectorB[3]);
void MatrixMultiply(float fMatrixA[3][3], float fMatrixB[3][3], float fMatrixR[3][3]);

