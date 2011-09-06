//============================================================================+
//
// $RCSfile: log.c,v $ (SOURCE FILE)
// $Revision: 1.5 $
// $Date: 2009/10/24 16:52:08 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             Log manager
//
//  CHANGES     UARTSend replaced with UART0Send
//
//============================================================================*/

#include "math.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "tff.h"
#include "DCM.h"
#include "tick.h"
#include "uartdriver.h"

#include "log.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define LOG_TO_FILE 0

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

//
// Tiny FAT file system related variables.
//
#if LOG_TO_FILE == 1
VAR_STATIC const char g_szFileName[16] = "log.txt"; // File name
VAR_STATIC FATFS g_sFatFs;                          // FAT
VAR_STATIC FIL g_fFile;                             // File object 
VAR_STATIC BYTE g_Buff[256];                        // File data buffer
#endif

/*--------------------------------- Prototypes -------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief   Initialize log manager
///
/// \remarks 
///
//----------------------------------------------------------------------------
void 
LogInit( void ) 
{
#if LOG_TO_FILE == 1
    //
    // Enable SSI 0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Mount the file system, using logical disk 0.
    //
    if (FR_OK != f_mount(0, &g_sFatFs))
    {
      //
      // Open log file for write.
      //
      if (FR_OK != f_open(&g_fFile, g_szFileName, FA_WRITE))
      {
      }
    }
#endif
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Outputs DCM Matrix
/// \RETURN      -
/// \REMARKS     
///
///
///----------------------------------------------------------------------------
void
Log(void)
{
    unsigned char szString[32];
    long lTemp;

    UART0Send("rmat:\nrow1:", 11);
    int j = 0;
    for (int x = 0; x < 3; x++)
    {
        lTemp = (long)ceil(DCM_Matrix[0][x] * 32767.0f);
        szString[j++] = ' ';
        if (lTemp < 0) 
        {
          lTemp = -lTemp;
          szString[j++] = '-';
        }
        else
        {
          szString[j++] = ' ';
        }
        szString[j++] = '0' + ((lTemp / 10000) % 10);
        szString[j++] = '0' + ((lTemp / 1000) % 10);
        szString[j++] = '0' + ((lTemp / 100) % 10); 
        szString[j++] = '0' + ((lTemp / 10) % 10);  
        szString[j++] = '0' +  (lTemp % 10);        
    }
    UART0Send((const unsigned char *)szString, 21);

#if LOG_TO_FILE == 1
    int i = 0;
    for (int j = 0; j < 21; j++)
    {
        g_Buff[i++] = szString[j];
    }
    g_Buff[i++] = '\n';
#endif

    UART0Send("\nrow2:", 6);
    j = 0;
    for (int x = 0; x < 3; x++)
    {
        lTemp = (long)ceil(DCM_Matrix[1][x] * 32767.0f);
        szString[j++]  = ' ';
        if (lTemp < 0) 
        {
          lTemp = -lTemp;
          szString[j++] = '-';
        }
        else
        {
          szString[j++] = ' ';
        }
        szString[j++]  = '0' + ((lTemp / 10000) % 10);
        szString[j++]  = '0' + ((lTemp / 1000) % 10);
        szString[j++]  = '0' + ((lTemp / 100) % 10); 
        szString[j++]  = '0' + ((lTemp / 10) % 10);  
        szString[j++]  = '0' +  (lTemp % 10);        
    }
    UART0Send((const unsigned char *)szString, 21);

#if LOG_TO_FILE == 1
    for (j = 0; j < 21; j++)
    {
        g_Buff[i++] = szString[j];
    }
    g_Buff[i++] = '\n';
#endif

    UART0Send("\nrow3:", 6);
    j = 0;
    for (int x = 0; x < 3; x++)
    {
        lTemp = (long)ceil(DCM_Matrix[2][x] * 32767.0f);
        szString[j++]  = ' ';
        if (lTemp < 0) 
        {
          lTemp = -lTemp;
          szString[j++] = '-';
        }
        else
        {
          szString[j++] = ' ';
        }
        szString[j++]  = '0' + ((lTemp / 10000) % 10);
        szString[j++]  = '0' + ((lTemp / 1000) % 10);
        szString[j++]  = '0' + ((lTemp / 100) % 10); 
        szString[j++]  = '0' + ((lTemp / 10) % 10);  
        szString[j++]  = '0' +  (lTemp % 10);        
    }
    UART0Send((const unsigned char *)szString, 21);
    UART0Send("\n", 1);

#if LOG_TO_FILE == 1
    for (j = 0; j < 21; j++)
    {
        g_Buff[i++] = szString[j];
    }
    g_Buff[i++] = '\n';
    g_Buff[i] = '\n';

    WORD wWritten;
    static unsigned long ulSamples = 1;
    if (ulSamples != 0)
    {
        if ((ulSamples % 5) == 0)
        {
            f_write(&g_fFile, g_Buff, i, &wWritten);
            if (wWritten != i)
            {
                ulSamples = 1001;
            }
        }
        if ((ulSamples >= 1000) || (HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS)))
        {
            ulSamples = 0;
            f_close(&g_fFile);
            HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;
        }
        else
        {
            ulSamples++;
        }
    }
#endif
}
