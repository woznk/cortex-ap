//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Vector algebra header file
///
/// \file
///
//  Change (Lint) pointer arguments made const when possible
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

float VectorDotProduct(const float fVectorA[3], const float fVectorB[3]);
void VectorCrossProduct(float fCrossP[3], const float fVectorA[3], const float fVectorB[3]);
void VectorScale(float fScaledV[3], const float fVector[3], const float fScale);
void VectorAdd(float fSumV[3], const float fVectorA[3], const float fVectorB[3]);
void MatrixMultiply(const float fMatrixA[3][3], const float fMatrixB[3][3], float fMatrixR[3][3]);

