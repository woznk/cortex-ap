/*============================================================================+
/
/ $HeadURL: $
/ $Revision: $
/ $Date:  $
/ $Author: $
/
/ \brief  Log manager
/
/ \file
/
/  Change added missing parenthesis
/
/============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "ff.h"
#include "bmp085_driver.h"
#include "ppmdriver.h"
#include "nav.h"
#include "globals.h"
#include "log.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef    VAR_STATIC
#   define VAR_STATIC static
#endif

#define MAX_SAMPLES 20000U   /*!< Max number of samples that can be written */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*VAR_GLOBAL xQueueHandle xLog_Queue; */ /* Queue for log messages */

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t uc_Index = 0U;
VAR_STATIC bool b_File_Ok = FALSE;
VAR_STATIC UINT wWritten;
VAR_STATIC uint8_t * p_Data;
VAR_STATIC portTickType Last_Wake_Time;
VAR_STATIC int32_t l_Value[8];                  /*!< sample values */
VAR_STATIC uint16_t ui_Samples = 0U;            /*!< sample counter */
VAR_STATIC uint8_t sz_String[48];               /*!< generic string */
VAR_STATIC uint8_t sz_File[16] = "log0.txt";    /*!< file name */

/*--------------------------------- Prototypes -------------------------------*/

static void log_write(int32_t *data, uint8_t num);
static __inline void log_raw_gps(void);
static __inline void log_position(void);

/*--------------------------------- Functions --------------------------------*/

/*----------------------------------------------------------------------------
/
/ \brief   log task
/ \return  -
/ \remarks task initially waits 20 sec to avoid contention between log file
/          and path file, read by navigation task.
/
/----------------------------------------------------------------------------*/
void Log_Task( void *pvParameters ) {

    uint8_t j;
    bool b_found = TRUE;
/*    xLog_Message message; */

    (void) pvParameters;

    Last_Wake_Time = xTaskGetTickCount();

    /* wait 10 sec for navigation task to complete reading waypoint file */
    vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ * (portTickType)10);

    /* wait until file system mounted and GPS got fix */
    while ((Gps_Fix() != GPS_FIX) || (!b_FS_Ok)) {
    }

    /* Search last log file */
    for (j = 0U; (j < 10U) && b_found; j++) {   /* */
        sz_File[3U] = '0' + j;                  /* Append file number */
        if (FR_OK == f_open(&st_File, (const XCHAR *)sz_File, FA_WRITE)) {
            b_found = TRUE;                     /* File exist */
            ( void )f_close(&st_File);          /* Close file */
        } else {                                /* */
            b_found = FALSE;                    /* File doesn't exist */
        }
    }

    /* Open new log file */
    if (!b_found) {                             /* File doesn't exist */
        if (FR_OK == f_open(&st_File, 
                            (const XCHAR *)sz_File,
                            (BYTE)(FA_WRITE|FA_CREATE_ALWAYS))) {
            b_File_Ok = TRUE;                   /* File succesfully open */
        }
    }

    /* wait until RC is turned on */
    while (PPMGetMode() == MODE_RTL) {
    }

    if (PPMGetMode() == MODE_MANUAL) {  /* mode manual */
        log_raw_gps();                  /* log GPS data */
    } else {                            /* mode stab or nav */
        log_position();                 /* log position */
    }
/*
    while ( xLog_Queue != 0 ) {
        while (xQueueReceive( xLog_Queue, &message, portMAX_DELAY ) != pdPASS) {
        }
        log_write(message.pcData, message.uc_length);
    }
*/
}


/*----------------------------------------------------------------------------
/
/ \brief   writes an array of 32 bit variables to SD card
/ \param   data = pointer to variable array
/ \param   num = number of variables to be written
/ \return  -
/ \remarks increases counter of samples at each function call.
/          closes log file if there is no file space, or if sample counter
/          exceeded maximum.
/
/----------------------------------------------------------------------------*/
static void log_write(int32_t *data, uint8_t num)
{
    int32_t l_temp;
    uint16_t j = 0U;
    uint8_t digit, i, k;

    for (i = 0U; i < num; i++) {                            /* repeat for all variables */
        l_temp = *data++;                                   /* get variable value */
        sz_String[j++] = ' ';                               /* separate with blank space */
        for (k = 0U; k < 8U; k++) {                         /* repeat for all 8 digits */
            l_temp = (uint32_t)l_temp >> ((7U - k) * 4U);
            digit = (uint8_t)(l_temp & 0x0000000F);         /* extract digit */
            if (digit < 10U) {                              /* digit is numerical */
                sz_String[j++] = digit + '0';               /* write number */
            } else {                                        /* digit is alphabetical */
                sz_String[j++] = (digit - 10) + 'A';        /* write letter */
            }
        }
    }
    sz_String[j++] = '\n';                                  /* terminate line */
    if (b_File_Ok) {                                        /* file is open */
        ( void )f_write(&st_File, sz_String, j, &wWritten); /* write line */
        if ((j != wWritten) ||                              /* no file space */
            (ui_Samples >= MAX_SAMPLES)) {                  /* too many samples */
            ( void )f_close(&st_File);                      /* close file */
            b_File_Ok = FALSE;                              /* halt logging */
        } else {                                            /* write successfull */
            ui_Samples++;                                   /* update sample counter */
        }
    }
}

/*----------------------------------------------------------------------------
/
/ \brief
/ \param   -
/ \return  -
/ \remarks
/
/
/
/----------------------------------------------------------------------------*/
static __inline void log_raw_gps(void) {

    for (;;) {
        /* halt if file not open GPS buffer empty */
        while ((Gps_Buffer_Index() == uc_Index) || (!b_File_Ok)) {
        }

        /* get GPS buffer pointer */
        p_Data = Gps_Buffer_Pointer();

        /* log raw GPS buffer */
        (void)f_write(&st_File, p_Data, (BUFFER_LENGTH / 2U), &wWritten);

        /* update index */
        uc_Index = (uc_Index + (BUFFER_LENGTH / 2U)) % BUFFER_LENGTH;

        if ((PPMGetMode() == MODE_RTL) ||         /* RC turned off */
            ((BUFFER_LENGTH / 2U) != wWritten)) { /* no file space */
            ( void )f_close(&st_File);            /* close file */
            b_File_Ok = FALSE;                    /* halt GPS logging */
        }
    }
}

/*----------------------------------------------------------------------------
/
/ \brief
/ \param   -
/ \return  -
/ \remarks
/
/
/
/----------------------------------------------------------------------------*/
static __inline void log_position(void) {

    for (;;) {

        /* wake up once every second */
        vTaskDelayUntil(&Last_Wake_Time, configTICK_RATE_HZ);

        /* halt if file not open */
        while (!b_File_Ok) {
        }

        l_Value [0] = Gps_Latitude();           /* get latitude */
        l_Value [1] = Gps_Longitude();          /* get longitude */
        l_Value [2] = (int32_t)Gps_Alt_M();     /* get GPS altitude */
        l_Value [3] = BMP085_Get_Altitude();    /* get baro altitude */
        log_write(l_Value, 4U);                 /* log position */

        if (PPMGetMode() == MODE_RTL) {         /* RC turned off */
            ( void )f_close(&st_File);          /* close file */
            b_File_Ok = FALSE;                  /* halt position logging */
        }
    }
}

/*----------------------------------------------------------------------------
/
/ \brief   float to ASCII conversion
/ \param   f_val = value to be converted
/ \param   p_string = pointer to destination string
/ \return  length of resulting string
/ \remarks -
/
/----------------------------------------------------------------------------*/
uint8_t ftoa (float f_val, uint8_t * p_string) {

    int32_t l_temp, l_sign = (int32_t)1;
    uint8_t uc_temp[10];
    uint8_t uc_length, j = 0U;

    f_val *= 1000000.0f;
    l_temp = (int32_t)f_val;

    if (l_temp < (int32_t)0) {
        l_sign = (int32_t)-1;
        l_temp = -l_temp;
    }

    if (l_temp > (int32_t)1000000) {
        l_temp = (int32_t)1000000;
    }

    do {
        uc_temp[j++] = '0' + (uint8_t)(l_temp % 10);
        l_temp /= 10;
    } while (l_temp != 0);

    if (l_sign == -1) {
        uc_temp[j++] = '-';
    }

    uc_length = j;

    while (j) {
        *p_string++ = uc_temp[--j];
    }

    return uc_length;
}

