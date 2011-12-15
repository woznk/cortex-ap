//============================================================================+
//
// Copyright (c) 2007 Lorenzo Fraccaro     <l.fraccaro@email.it>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.
//
// See README and COPYING for more details.
//
// $RCSfile: simulation.c,v $ (SOURCE FILE)
// $Revision: 1.3 $
// $Date: 2009/10/31 15:11:41 $
//
//  LANGUAGE    C
/// \brief      Navstix simulation file
/// \file
///                                 PITCH+
//                      - >           - > 
//                    /    Z+       /       
//              YAW+ |      (.)----| -----> Y+
//                    \      |      \     
//                      -    |        -   
//                           |         
//                           |   ^
//                       \   |   /  ROLL+ 
//                         -___-    
//                           |
//                           |
//                           V  X+
//              
//  CHANGES      corrected naming of reference axes
//               added sensor signs
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "uartdriver.h"
#include "simulation.h"

//------------ Definitions -------------------------------------------------

#ifdef VAR_STATIC
#   undef VAR_STATIC
#endif
#define VAR_STATIC static

#ifdef VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define SIMULATOR_DEBUG 0

#if (SIMULATOR_DEBUG == 0)
#   define SimChar UART1GetChar
#else
#   define SimChar SimulatedSimChar
#endif

/****************************************************************************
*
* An offset is added to convert the usPosition that crrcsim outputs
* into actual airfield coordinates
*
****************************************************************************/
#define LAT_OFFSET	( 45.540250f ) // latitude offset
#define LON_OFFSET	( 11.433442f ) // longitude offset

// ---- Constants and Types -------------------------------------------------

#if (SIMULATOR_DEBUG == 1)
const char s_pcSentence[] =
"S=100,200,300,400,500,600\nS=560,112,-12,12345,0,1023\nS=512,512,512,512,512,512,512,512\nS=512,512,512,\nS=512,512,512,512,512,512\n";
#endif

// ---- Private Variables ---------------------------------------------------

//VAR_STATIC unsigned short g_usRudder;             // Rudder
//VAR_STATIC unsigned short g_usAileron;            // Aileron
//VAR_STATIC unsigned short g_usElevator;           // Elevator
//VAR_STATIC unsigned short g_usThrottle;           // Throttle
//VAR_STATIC long lLongitude;
//VAR_STATIC long lLatitude;
VAR_STATIC float pfSimSensorData[8];      // Simulator sensor data
VAR_STATIC float pfSimSensorOffset[8];    // Simulator sensor offsets
VAR_STATIC unsigned long ulChCounter = 1;

//
// Used to change the polarity of the sensors
//
VAR_STATIC float pfSensorSign[8] = {
   1.0f, // 0 : accel X
  -1.0f, // 1 : accel Z
   1.0f, // 2 : accel Y
   1.0f, // 3 : omega X 
  -1.0f, // 4 : omega Y 
   1.0f, // 5 : omega Z 
   1.0f, // 6 : 
   1.0f  // 7 : 
};

// ---- Private Function Prototypes -----------------------------------------

tBoolean SimulatedSimChar ( char *ch );

// ---- Functions -----------------------------------------------------------


//----------------------------------------------------------------------------
//
/// \brief   parse FlightGear protocol
///
/// \returns TRUE if new data are available, FALSE otherwise
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean Sim_ParseFlightGear ( void )
{
    static unsigned char ucIndex = 0;
    static unsigned char ucStatus = 0;
    static float fTemp = 0.0f;
    static float fSign = 1.0f;

    char c;
    tBoolean bResult = false;

    while (SimChar(&c))
    {
        if (ulChCounter != 0) { ulChCounter++; }

        switch (ucStatus)
        {
            case 0:                                     // preamble
                if (c == 'S') { ucStatus++; }
                break;
            case 1:
                if (c == '=') { ucStatus++; } else { ucStatus = 0; }
                break;
            case 2:
                if (c == '-')
                {
                    ucStatus++;
                    fSign = -1.0f;
                }
                else if ((c >= '0') && (c <= '9'))
                {
                    ucStatus++;
                    fSign = 1.0f;
                    fTemp = fTemp * 10 + (float)(c - '0');
                }
                else
                {
                    ucStatus = 0;
                }
                break;
            case 3:
                if ((c >= '0') && (c <= '9'))
                {
                    fTemp = fTemp * 10 + (float)(c - '0');
                }
                else if ((c == ',') || (c == '\n'))
                {
                    if (fTemp < 0.0f) { fTemp = 0.0f; }
                    if (fTemp > 1023.0f) { fTemp = 1023.0f; }
                    pfSimSensorData[ucIndex] = fTemp * fSign;
                    ucStatus = (c == '\n') ? 0 : 2;
                    ucIndex =  (c == '\n') ? 0 : (ucIndex + 1);
                    if (ucIndex > 5) {ucIndex = 0; ucStatus = 0; }
                    fTemp = 0.0f;
                }
                else
                {
                    ucStatus = 0;
                }
                break;
            default:
                ucStatus = 0;
                break;
        }
    }
    if (ulChCounter > 1000)
    {
        for (int j = 0; j < 6 ; j++)
        {
            pfSimSensorOffset[j] = pfSimSensorData[j];
        }
        pfSimSensorOffset[1] += 102.0f;
        ulChCounter = 0;
        bResult = true;
    }

    return bResult;
}


///----------------------------------------------------------------------------
///
///  DESCRIPTION Interface to simulator data.
/// \RETURN
/// \REMARKS
///
///----------------------------------------------------------------------------
float
Sim_GetData(int n)
{
    float fData;
    switch (n)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            fData = (pfSimSensorData[n] - pfSimSensorOffset[n]) * pfSensorSign[n];
            break;
        default:
            fData = 0.0f;
            break;
    }
    return fData;
}

#if (SIMULATOR_DEBUG == 1)
//----------------------------------------------------------------------------
//
/// \brief   get a character from preloaded sentence
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean
SimulatedSimChar ( char *ch )
{
    char c;
    VAR_STATIC unsigned char j = 0;

    c = s_pcSentence[j];
    if (c == 0)
    {
        j = 0;
    }
    else
    {
        j++;
    }
    *ch = c;
    return true;
}
#endif
