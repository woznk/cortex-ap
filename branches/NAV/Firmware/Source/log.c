//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief  Log manager
///
/// \file
///
//  CHANGES log file closed when RC is switched off
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
#define MAX_SAMPLES        20000 // Max number of samples that can be written

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xLog_Queue;

/*----------------------------------- Locals ---------------------------------*/


VAR_STATIC uint8_t szFileName[16] = "log0.txt";     // File name
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC char szString[48];                       //
VAR_STATIC bool bFileOk = FALSE;                    // File status
VAR_STATIC uint16_t uiSamples;


/*--------------------------------- Prototypes -------------------------------*/

static void Log_Write(uint16_t *data, uint8_t num);

/*--------------------------------- Functions --------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Log_Task( void *pvParameters ) {

    uint8_t j;
    xLog_Message message;

    uiSamples = 0;
    bFileOk = TRUE;                             //

    // Open log file
    if (FR_OK == f_mount(0, &stFat)) {          // Mount file system
        for (j = 0; (j < 10) && bFileOk; j++) { // Search last log file
            szFileName[3] = '0' + j;            // Append file number
            if (FR_OK == f_open(&stFile, (const XCHAR *)szFileName, FA_WRITE)) {
                bFileOk = TRUE;                 // File exist
                f_close(&stFile);               // Close file
            } else {                            //
                bFileOk = FALSE;                // File doesn't exist
            }
        }
        if (!bFileOk) {                         // File doesn't exist
            if (FR_OK == f_open(&stFile, (const XCHAR *)szFileName, FA_WRITE|FA_CREATE_ALWAYS)) {
                bFileOk = TRUE;                 // File succesfully open
            } else {                            // Error opening file
                bFileOk = FALSE;                // Halt file logging
            }
        } else {
            bFileOk = FALSE;                    // Halt file logging
        }
    } else {                                    // Error mounting FS
        bFileOk = FALSE;                        // Halt file logging
    }

    while ( xLog_Queue != 0 ) {
        while (xQueueReceive( xLog_Queue, &message, portMAX_DELAY ) != pdPASS) {
        }
        Log_Write(message.pcData, message.ucLength);
    }
}


///----------------------------------------------------------------------------
///
/// \brief
/// \remarks
///
///
///----------------------------------------------------------------------------
static void Log_Write(uint16_t *data, uint8_t num)
{
    long l_temp;
    uint8_t digit, mode, i, j = 0;
    UINT wWritten;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        szString[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = (l_temp & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
    }
    szString[j++] = '\n';
    mode = PPMGetMode();
    if (bFileOk) {                                          //
        f_write(&stFile, szString, j, &wWritten);           // write
        if (                                                // button pressed
            (j != wWritten) ||                              // no file space
            (mode == MODE_RTL) ||                           // RC turned off
            (uiSamples >= MAX_SAMPLES)) {                   // too many samples
            f_close(&stFile);                               // close file
            bFileOk = FALSE;                                // halt file logging
        } else {                                            // write successfull
            uiSamples++;                                    // update sample counter
        }
    }
}


/*
uint8_t ftoa (float fVal, uint8_t *pucString) {

    long lTemp, lSign = 1;
    uint8_t pucTemp[10];
    uint8_t ucLength, j = 0;

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
