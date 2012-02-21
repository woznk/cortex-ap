//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
///
//
//  CHANGES     cosmetics
//
//============================================================================*/

#include "cordic.h"

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

VAR_STATIC const long Phase_Table[15] =
{           // degrees
   45000,   // 45
   26565,   // 26,56505118
   14036,   // 14,03624347
    7125,   // 7,125016349
    3576,   // 3,576334375
    1789,   // 1,789910608
     895,   // 0,89517371
     447,   // 0,447614171
     223,   // 0,2238105
     111,   // 0,111905677
      55,   // 0,055952892
      27,   // 0,027976453
      13,   // 0,013988226
       7,   // 0,006994113
       3    // 0,0034970565
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/


//----------------------------------------------------------------------------
//
/// \brief   CORDIC based arc tangent computation
///
/// \remarks
///
//----------------------------------------------------------------------------
long cordic_atan(long I, long Q)
{
  int step;
  long tmp_I, acc_phase;

  acc_phase = 0;

  if (I < 0)   {                        // rotate by an initial +/- 90°
    tmp_I = I;
    if (Q > 0) {
       I = Q;		                    // +90°
       Q = -tmp_I;
       acc_phase = -90000;
    } else  {
       I = -Q;		                    // -90°
       Q = tmp_I;
       acc_phase = 90000;
    }
  }

  if (Q < 0) {
     tmp_I = (I > -Q) ? I : -Q;
  } else {
     tmp_I = (I > Q) ? I : Q;
  }

  while ((tmp_I < 8192) && (tmp_I > 0)) { // Scale to reach at least 13 bits
      tmp_I *= 2;
      I *= 2;
      Q *= 2;
  }

  for (step = 0; step < 15; step++) {    // rotate using "1 + jK" factors
    tmp_I = I;
    if (Q >= 0)  {                      // phase is positive: do negative rotation
      I += (Q >> step);
      Q -= (tmp_I >> step);
      acc_phase -= Phase_Table[step];
    } else {                                   // phase is negative: do positive rotation
      I -= (Q >> step);
      Q += (tmp_I >> step);
      acc_phase += Phase_Table[step];
    }
  }
  return -acc_phase;
}

#define COSCALE 607             // 0.607252 : step =  0, 1, 2, ...

//----------------------------------------------------------------------------
//
/// \brief   CORDIC based rotation
///
/// \remarks assumes that 0° <= theta <= 360°
///
//----------------------------------------------------------------------------
void cordic_rotate( int *px, int *py, int theta )
{
    unsigned char step;
    int x = *px, y = *py, xtemp, sign = 1;

    if (theta > 270)
    {
        theta = theta - 360;
    }
    if (theta > 90)
    {
        theta = 180 - theta;
        sign = -1;
    }
    theta = theta * 100;

    x <<= 2;    // Scale coordinates
    y <<= 2;    // DEPENDS ON COORDINATE VALUES !!!

    for (step = 0; step < 13; step++)
    {
        if (theta < 0)
        {
            xtemp = x + (y >> step); // Left shift when i = -1
            y = y - (x >> step);
            x = xtemp;
            theta += Phase_Table[step];
        }
        else
        {
            xtemp = x - (y >> step);
            y = y + (x >> step);
            x = xtemp;
            theta -= Phase_Table[step];
        }
    }

    x >>= 2;
    y >>= 2;

    *px = (x * COSCALE) / 1000;         // Compensate for CORDIC enlargement
    *py = (sign * y * COSCALE) / 1000;  //
}
