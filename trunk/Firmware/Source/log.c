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
//  CHANGES Log_Send() modified to send multiple words
//
//============================================================================*/

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "math.h"
//#include "telemetry.h"
//#include "config.h"
//#include "gps.h"
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

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char szString[48];
VAR_STATIC const char szFileName[16] = "log.txt";   // File name
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
VAR_STATIC WORD wWriteIndex = 0;                    // File buffer write index
VAR_STATIC bool bFileOk = FALSE;                    // File status

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
void
Log_Init( void ) {

    USART_InitTypeDef USART_InitStructure;

    // Initialize USART1 structure
    USART_InitStructure.USART_BaudRate = 38400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // Configure USART1
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1 interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1,USART_IT_TXE,ENABLE);

    // Enable the USART1
    USART_Cmd(USART1, ENABLE);

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
/// \brief   sends data via USART 1
/// \remarks
///
///
///----------------------------------------------------------------------------
void
Log_Send(uint16_t *data, uint8_t num)
{
    long l_temp;
    uint8_t digit, i, j = 0;

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
    szString[j] = '\r';

    for (j = 0; j < (i * 5) + 2; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, szString[j]);
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
Log_PutChar ( char c ) {

    UINT wWritten;
    static unsigned long ulSamples;

    if (wWriteIndex < FILE_BUFFER_LENGTH) { // Provided buffer is not full
        pcBuffer[wWriteIndex++] = c;        // Save character in buffer
    }
    if ((c == '\r') && bFileOk) {                           // End of line
        f_write(&stFile, pcBuffer, wWriteIndex, &wWritten); // Write
        if (/* (HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS)) || */// button pressed
            (wWriteIndex != wWritten) ||                    // No file space
            (ulSamples >= MAX_SAMPLES)) {                   // Too many samples
            f_close(&stFile);                               // close file
            bFileOk = FALSE;                                // Halt file logging
/*            HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;   // clear button flag*/
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

    while ((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET) && (j < 47)) {
      USART_SendData(USART1, szString[j++]);
    }
    if (j == 47) {
      j = 0;
      for (y = 0; y < 3; y++) {
        for (x = 0; x < 3; x++) {
            lTemp = (long)ceil(/*DCM_Matrix[y][x] * */32767.0f);
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
/*
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
