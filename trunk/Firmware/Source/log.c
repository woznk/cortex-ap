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
//  Change added log of NMEA sentences
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "ff.h"
#include "ppmdriver.h"
#include "nav.h"
#include "globals.h"
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

#define FILE_BUFFER_LENGTH 128      //!< Length of file buffer
#define MAX_SAMPLES        20000    //!< Max number of samples that can be written

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xLog_Queue; //!< Queue for log messages

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC bool b_File_Ok = FALSE;
VAR_STATIC uint8_t * p_Data;
VAR_STATIC uint16_t ui_Samples = 0;             //!< sample counter
VAR_STATIC uint8_t sz_String[48];               //!< generic string
VAR_STATIC uint8_t sz_File[16] = "log0.nmea";   //!< file name

/*--------------------------------- Prototypes -------------------------------*/

static void log_write(uint16_t *data, uint8_t num);

/*--------------------------------- Functions --------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   log task
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Log_Task( void *pvParameters ) {

    uint8_t j;
    bool b_found = TRUE;
    UINT wWritten;
//    xLog_Message message;
    portTickType Last_Wake_Time;

    Last_Wake_Time = xTaskGetTickCount();
    vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ * 20);

    while ((!b_FS_Ok) ||                        // wait until file system mounted
           (Gps_Fix() != GPS_FIX)) {            // and GPS got fix
    }

    // Search last log file
    for (j = 0; (j < 10) && b_found; j++) {     //
        sz_File[3] = '0' + j;                   // Append file number
        if (FR_OK == f_open(&st_File, (const XCHAR *)sz_File, FA_WRITE)) {
            b_found = TRUE;                     // File exist
            ( void )f_close(&st_File);          // Close file
        } else {                                //
            b_found = FALSE;                    // File doesn't exist
        }
    }

    // Open new log file
    if (!b_found) {                             // File doesn't exist
        if (FR_OK == f_open(&st_File, (const XCHAR *)sz_File, FA_WRITE|FA_CREATE_ALWAYS)) {
            b_File_Ok = TRUE;                   // File succesfully open
        }
    }

    j = 0;

    while (1) {
        while (!b_File_Ok);                                             // halt if file not open
        while (Gps_Buffer_Index() == j);                                // halt if GPS buffer empty
        p_Data = Gps_Buffer_Pointer();                                  // get GPS buffer pointer
        (void)f_write(&st_File, p_Data, (BUFFER_LENGTH / 2), &wWritten);// write
        j = (j + (BUFFER_LENGTH / 2)) % BUFFER_LENGTH;                  // update index

        if ((PPMGetMode() == MODE_RTL) ||                               // RC turned off
            ((BUFFER_LENGTH / 2) != wWritten)) {                        // no file space
            ( void )f_close(&st_File);                                  // close file
            b_File_Ok = FALSE;                                          // halt GPS logging
        }
    }
/*
    while ( xLog_Queue != 0 ) {
        while (xQueueReceive( xLog_Queue, &message, portMAX_DELAY ) != pdPASS) {
        }
        log_write(message.pcData, message.uc_length);
    }
*/
}


///----------------------------------------------------------------------------
///
/// \brief   write some 16 bit variables to SD card
/// \param   data = pointer to variable
/// \param   num = number of variables to be written
/// \return  -
/// \remarks increases counter of samples at each function call.
///          closes log file if there is no file space, or if sample counter
///          exceeded maximum or if RC was temporarily shut off.
///
///----------------------------------------------------------------------------
static void log_write(uint16_t *data, uint8_t num)
{
    uint32_t l_temp;
    uint8_t digit, mode, i, j = 0;
    UINT wWritten;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        sz_String[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        sz_String[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        sz_String[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        sz_String[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = (l_temp & 0x0000000F);
        sz_String[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
    }
    sz_String[j++] = '\n';
    mode = PPMGetMode();
    if (b_File_Ok) {                                        //
        ( void )f_write(&st_File, sz_String, j, &wWritten); // write
        if (                                                // button pressed
            (j != wWritten) ||                              // no file space
            (mode == MODE_RTL) ||                           // RC turned off
            (ui_Samples >= MAX_SAMPLES)) {                  // too many samples
            ( void )f_close(&st_File);                      // close file
            b_File_Ok = FALSE;                              // halt file logging
        } else {                                            // write successfull
            ui_Samples++;                                   // update sample counter
        }
    }
}


///----------------------------------------------------------------------------
///
/// \brief   float to ASCII conversion
/// \param   f_val = value to be converted
/// \param   p_string = pointer to destination string
/// \return  length of resulting string
/// \remarks -
///
///----------------------------------------------------------------------------
uint8_t ftoa (float f_val, uint8_t * p_string) {

    int32_t l_temp, l_sign = 1L;
    uint8_t uc_temp[10];
    uint8_t uc_length, j = 0;

    l_temp = (int32_t)(f_val * 1000000.0f);

    if (l_temp < 0L) {
        l_sign = -1L;
        l_temp = -l_temp;
    }

    if (l_temp > 1000000L) {
        l_temp = 1000000L;
    }

    do {
        uc_temp[j++] = '0' + (uint8_t)(l_temp % 10L);
        l_temp /= 10L;
    } while (l_temp != 0L);

    if (l_sign == -1L) {
        uc_temp[j++] = '-';
    }

    uc_length = j;

    while (j) {
        *p_string++ = uc_temp[--j];
    }

    return uc_length;
}

