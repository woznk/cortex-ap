//============================================================================+
//
// $RCSfile: log.c,v $ (SOURCE FILE)
// $Revision: 1.10 $
// $Date: 2011/01/19 18:31:28 $
// $Author: Lorenz $
//
/// \brief    Log manager
///
/// \file
///
//  CHANGES funzione Log_DCM(): resa non sospensiva, semplificato il formato 
//          per l'invio della matrice DCM 
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

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char szString[48];
VAR_STATIC const char szFileName[16] = "log.txt";   // File name
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object 
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
VAR_STATIC WORD wWriteIndex = 0;                    // File buffer write index
VAR_STATIC tBoolean bFileOk = false;                // File status

/*--------------------------------- Prototypes -------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief   Initialize log manager
///
/// \remarks opens log file for writing
///
//----------------------------------------------------------------------------
void 
Log_Init( void ) {

    if (FR_OK == f_mount(0, &stFat)) { 
        if (FR_OK == f_open(&stFile, szFileName, FA_WRITE)) { 
            bFileOk = true;                     // File succesfully open
        } else {                                // Error opening file
            bFileOk = false;                    // Halt file logging
        }
    } else {                                    // Error mounting FS
        bFileOk = false;                        // Halt file logging
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Put characters to log file
///
/// \remarks Characters are saved in a buffer and written to file when EOL is 
///          received
///
//----------------------------------------------------------------------------
void 
Log_PutChar( char c ) {
  
    WORD wWritten;
    static unsigned long ulSamples;
    
    if (wWriteIndex < FILE_BUFFER_LENGTH) { // Provided buffer is not full
        pcBuffer[wWriteIndex++] = c;        // Save character in buffer
    }
    if ((c == '\r') && bFileOk) {                           // End of line
        f_write(&stFile, pcBuffer, wWriteIndex, &wWritten); // Write
        if ((wWriteIndex != wWritten) ||                    // No file space
            (ulSamples >= MAX_SAMPLES) ||                   // Too many samples
            (HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS))) {   // button pressed
            bFileOk = false;                                // Halt file logging
            f_close(&stFile);                               // close file
            HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;   // clear button flag
        } else {                                            // Write successfull
            ulSamples++;                                    // Update sample counter
            wWriteIndex = 0;                                // Empty buffer
        }
    }
}


///----------------------------------------------------------------------------
///
///  DESCRIPTION Outputs DCM matrix
/// \RETURN      -
/// \REMARKS     
///
///
///----------------------------------------------------------------------------
void
Log_DCM(void)
{   
    static int j = 47;
    unsigned char ucDigit;
    long lTemp;
    int x, y;
    
    while ((j < 47) && UARTSpaceAvail(UART1_BASE)) {
      UARTCharPutNonBlocking(UART1_BASE, szString[j++]);
    }
    if (j == 47) {
      j = 0;
      for (y = 0; y < 3; y++) {
        for (x = 0; x < 3; x++) {
            lTemp = (long)ceil(DCM_Matrix[y][x] * 32767.0f);
            szString[j++] = ' ';
            ucDigit = ((lTemp >> 12) & 0x0000000F);
            szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
            ucDigit = ((lTemp >> 8) & 0x0000000F);
            szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
            ucDigit = ((lTemp >> 4) & 0x0000000F); 
            szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
            ucDigit = (lTemp & 0x0000000F);        
            szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
        }
      }
      szString[j++] = '\n';
      szString[j] = '\r';
      j = 0;
    } 
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Outputs PPM input values
/// \RETURN      -
/// \REMARKS     
///
///
///----------------------------------------------------------------------------
void
Log_PPM(void)
{
    unsigned char szString[64];
    unsigned long ulTemp;
    int j = 0, chan = 0;

    for (chan = 0; chan < RC_CHANNELS; chan++) {
        ulTemp = PPMGetChannel(chan);
        szString[j++] = ' ';
        szString[j++] = '0' + ((ulTemp / 10000) % 10);
        szString[j++] = '0' + ((ulTemp / 1000) % 10);
        szString[j++] = '0' + ((ulTemp / 100) % 10); 
        szString[j++] = '0' + ((ulTemp / 10) % 10);  
        szString[j++] = '0' +  (ulTemp % 10);        
    }
    szString[j] = '\r';
    UART1Send((const unsigned char *)szString, RC_CHANNELS * 6 + 1);
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

void
Log_(void)
{
    unsigned char ucLen, szString[32];

    UART1Send ("!!!EX0:", 7);
    ucLen = ftoa(DCM_Matrix[0][0], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX1:", 5);
    ucLen = ftoa(DCM_Matrix[0][1], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX2:", 5);
    ucLen = ftoa(DCM_Matrix[0][2], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX3:", 5);
    ucLen = ftoa(DCM_Matrix[1][0], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX4:", 5);
    ucLen = ftoa(DCM_Matrix[1][1], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX5:", 5);
    ucLen = ftoa(DCM_Matrix[1][2], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX6:", 5);
    ucLen = ftoa(DCM_Matrix[2][0], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX7:", 5);
    ucLen = ftoa(DCM_Matrix[2][1], szString);
    UART1Send (szString, ucLen);
    UART1Send (",EX8:", 5);
    ucLen = ftoa(DCM_Matrix[2][2], szString);
    UART1Send (szString, ucLen);
    UART1Send (",***\r", 5);
}
*/