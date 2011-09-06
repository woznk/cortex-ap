//============================================================================+
//
// $RCSfile: cordic.c,v $ (SOURCE FILE)
// $Revision: 1.1 $
// $Date: 2009/10/12 17:04:18 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             
//
//  CHANGES     cordic_rotate(): input coords multiplied by 4, output coords divided by 4
//              cordic_rotate(): corrected inital adjustment of theta 
//              cordic_rotate(): change of y sign moved after rotation
//              
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

VAR_STATIC const int Phase_Table[13] = 
{           // degrees    
    4500,   // 45         
    2656,   // 26,56505118
    1403,   // 14,03624347
     712,   // 7,125016349
     357,   // 3,576334375
     178,   // 1,789910608
      89,   // 0,89517371 
      44,   // 0,447614171
      22,   // 0,2238105  
      11,   // 0,111905677
       5,   // 0,055952892
       2,   // 0,027976453
       1    // 0,013988226
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
int cordic_atan(int I, int Q)
{
  int step, tmp_I, acc_phase;

  acc_phase = 0;

  if (I < 0) 
  {                                     // rotate by an initial +/- 90°
    tmp_I = I;
    if (Q > 0)
    {
       I = Q;		                    // +90°
       Q = -tmp_I;
       acc_phase = -9000;
    }
    else 
    {
       I = -Q;		                    // -90°
       Q = tmp_I;
       acc_phase = 9000;
    }
  } 

  if (Q < 0)
  {
     tmp_I = (I > -Q) ? I : -Q;
  }
  else 
  {
     tmp_I = (I > Q) ? I : Q;
  }

  while ((tmp_I < 2048) && (tmp_I > 0)) // Scale to reach at least 11 bits
  {
      tmp_I *= 2;
      I *= 2;
      Q *= 2;
  }

  for (step = 0; step <= 12; step++)    // rotate using "1 + jK" factors
  {
    tmp_I = I;
    if (Q >= 0)
    {                                   // phase is positive: do negative rotation
      I += (Q >> step);
      Q -= (tmp_I >> step);
      acc_phase -= Phase_Table[step];
    }
    else
    {                                   // phase is negative: do positive rotation
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
