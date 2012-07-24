//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Vector algebra functions
///
//  CHANGES doxygen
//
//============================================================================*/

#include "FreeRTOS.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION Computes the dot product of two vectors
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
float
VectorDotProduct(float fVectorA[3], float fVectorB[3])
{
    float fDotP = 0.0f;
    uint8_t c;

    for ( c = 0; c < 3; c++ )
    {
        fDotP += fVectorA[c] * fVectorB[c];
    }
    return fDotP;
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Computes the cross product of two vectors
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
VectorCrossProduct(float fCrossP[3], float fVectorA[3], float fVectorB[3])
{
    fCrossP[0] = (fVectorA[1] * fVectorB[2]) - (fVectorA[2] * fVectorB[1]);
    fCrossP[1] = (fVectorA[2] * fVectorB[0]) - (fVectorA[0] * fVectorB[2]);
    fCrossP[2] = (fVectorA[0] * fVectorB[1]) - (fVectorA[1] * fVectorB[0]);
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Multiply the vector by a scalar.
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
VectorScale(float fScaledV[3], float fVector[3], float fScale)
{
    uint8_t c;

    for ( c = 0; c < 3; c++ )
    {
        fScaledV[c] = fVector[c] * fScale;
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Add two vectors.
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
VectorAdd(float fSumV[3], float fVectorA[3], float fVectorB[3])
{
    uint8_t c;

    for ( c = 0; c < 3; c++)
    {
        fSumV[c] = fVectorA[c] + fVectorB[c];
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Multiply two 3x3 matrices.
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
MatrixMultiply(float fMatrixA[3][3], float fMatrixB[3][3], float fMatrixR[3][3])
{
    uint8_t x, y, w;
    float fMatrixT[3];

    for ( x = 0; x < 3; x++ )
    {
        for ( y = 0; y < 3; y++ )
        {
            for ( w = 0; w < 3; w++ )
            {
                fMatrixT[w] = fMatrixA[x][w] * fMatrixB[w][y];
            }
            fMatrixR[x][y] = 0.0f;
            fMatrixR[x][y] = fMatrixT[0] + fMatrixT[1] + fMatrixT[2];
        }
    }
}

