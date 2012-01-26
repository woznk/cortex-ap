//============================================================================+
//
// $RCSfile: $ (SOURCE FILE)
// $Revision: $
// $Date: $
// $Author: Lorenz $
//
/// \brief  Log manager
///
/// \file
///
//  CHANGES Data transmission moved to telemetry task
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "math.h"
#include "ff.h"
#include "DCM.h"
#include "ppmdriver.h"
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

#define FILE_BUFFER_LENGTH 128
#define MAX_SAMPLES        1000 // Max number of samples that can be written

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xLog_Queue;

/*----------------------------------- Locals ---------------------------------*/

/*
VAR_STATIC const char szFileName[16] = "log.txt";   // File name
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
VAR_STATIC WORD wWriteIndex = 0;                    // File buffer write index
VAR_STATIC bool bFileOk = FALSE;                    // File status
*/

/*--------------------------------- Prototypes -------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief   Initialize log manager
///
/// \remarks opens log file for writing, configures USART1.
///          See http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///          for direct register initialization of USART 1
///
//----------------------------------------------------------------------------
void Log_Init( void ) {
/*
    // Open log file
    if (FR_OK == f_mount(0, &stFat)) {
        if (FR_OK == f_open(&stFile, szFileName, FA_WRITE|FA_CREATE_ALWAYS)) {
            bFileOk = TRUE;                     // File succesfully open
        } else {                                // Error opening file
            bFileOk = FALSE;                    // Halt file logging
        }
    } else {                                    // Error mounting FS
        bFileOk = FALSE;                        // Halt file logging
    }
*/
}


///----------------------------------------------------------------------------
///
/// \brief
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Log_Task( void *pvParameters )
{
}


//----------------------------------------------------------------------------
//
/// \brief   Put characters to log file
///
/// \remarks Characters are saved in a buffer and written to file when EOL is
///          received
///
//----------------------------------------------------------------------------
void Log_PutChar ( char c ) {
/*
    UINT wWritten;
    static unsigned long ulSamples;

    if (wWriteIndex < FILE_BUFFER_LENGTH) { // Provided buffer is not full
        pcBuffer[wWriteIndex++] = c;        // Save character in buffer
    }
    if ((c == '\r') && bFileOk) {                           // End of line
        f_write(&stFile, pcBuffer, wWriteIndex, &wWritten); // Write
        if (//(HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS)) ||  // button pressed
            (wWriteIndex != wWritten) ||                    // No file space
            (ulSamples >= MAX_SAMPLES)) {                   // Too many samples
            f_close(&stFile);                               // close file
            bFileOk = FALSE;                                // Halt file logging
//            HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;   // clear button flag
        } else {                                            // Write successfull
            ulSamples++;                                    // Update sample counter
            wWriteIndex = 0;                                // Empty buffer
        }
    }
*/
}


/*
unsigned char
ftoa (float fVal, unsigned char *pucString)
{
    long lTemp, lSign = 1;
    unsigned char pucTemp[10];
    unsigned char ucLength, j = 0;

    lTemp = (long)(fVal * 1000000.0f);
    if (lTemp < 0) { lSign = -1; lTemp = -lTemp; }
    if (lTemp > 1000000) { lTemp = 1000000; }
    do {
        pucTemp[j++] = '0' + (unsigned char)(lTemp % 10);
        lTemp /= 10;
    } while (lTemp != 0);
    if (lSign == -1) { pucTemp[j++] = '-'; }
    ucLength = j;
    while (j) {
        *pucString++ = pucTemp[--j];
    }
    return ucLength;
}

*/
