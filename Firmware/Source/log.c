/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief log
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

#include "ch.h"

#include "config.h"
#include "rc.h"
#include "gps.h"
#include "baro.h"
#include "ff.h"
#include "log.h"

/*--------------------------------- Definitions ------------------------------*/

#define MAX_SAMPLES     10000

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

extern bool_t fs_ready;

/*----------------------------------- Locals ---------------------------------*/

static uint8_t sz_log_file[10] = "log0.txt";    /* log file name */
static FIL file;                                /* file structure */
static UINT ui_written;                         /* counter of bytes written */
static bool_t b_file_ok = FALSE;                /* log file succesfully open */
static uint32_t ul_samples = 0;                 /* counter of samples written */

/*--------------------------------- Prototypes -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   open log file
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Init_Log ( void ) {

  uint8_t j;
  bool b_found = TRUE;

  /* Check file system */
  if (!fs_ready) {
     return;
  }

  /* Search last log file, if any */
  for (j = 0; (j < 10) && b_found; j++) {
    sz_log_file[3] = '0' + j;               /* Append file number */
    if (FR_OK == f_open(&file,
                        (const TCHAR *)sz_log_file,
                        FA_WRITE)) {
      b_found = TRUE;                       /* File exist */
      (void)f_close(&file);                 /* Close file */
    } else {
      b_found = FALSE;                      /* File doesn't exist */
    }
  }

  /* Open new log file */
  if (!b_found) {                           /* File doesn't exist */
    if (FR_OK == f_open(&file,
                        (const TCHAR *)sz_log_file,
                        FA_WRITE|FA_CREATE_ALWAYS)) {
      b_file_ok = TRUE;                     /* File succesfully open */
    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   writes a hexadecimal variable to SD card
 * @param   data = pointer to variable
 * @param   size = variable size, in bytes
 * @return  -
 * @remarks increases counter of samples at each function call,
 *          closes log file if there is no file space, or if sample counter
 *          exceeded maximum.
 *
 *----------------------------------------------------------------------------*/
void Log_Write_Var ( uint8_t *data, uint8_t size ) {

  uint8_t sz_string[10];
  uint8_t msb;
  uint8_t lsb;
  uint8_t k;
  uint8_t j = 0;

  if (size > 8) {
    size = 8;
  }

  sz_string[j++] = ' ';                         /* separate with blank space */
  for (k = 0; k < size; k++) {                  /* repeat for all digits */
    msb = *data++;
    lsb = msb & 0x0F;
    msb = (msb >> 4) & 0x0F;
    if (msb < 10) {                             /* digit is numerical */
      sz_string[j++] = msb + '0';               /* write number */
    } else {                                    /* digit is alphabetical */
      sz_string[j++] = (msb - 10) + 'A';        /* write letter */
    }
    if (lsb < 10) {                             /* digit is numerical */
      sz_string[j++] = lsb + '0';               /* write number */
    } else {                                    /* digit is alphabetical */
      sz_string[j++] = (lsb - 10) + 'A';        /* write letter */
    }
  }

  sz_string[j++] = ',';                         /* terminate line */
  if (b_file_ok) {                              /* file is open */
    f_write(&file, sz_string, j, &ui_written);  /* write line */
    if ((j != ui_written) ||                    /* no file space */
        (ul_samples >= MAX_SAMPLES)) {          /* too many samples */
      f_close(&file);                           /* close file */
      b_file_ok = FALSE;                        /* halt logging */
    } else {                                    /* write successfull */
      ul_samples++;                             /* update sample counter */
    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   writes a character to SD card
 * @param   ch = character
 * @return  -
 * @remarks closes log file if there is no file space
 *
 *----------------------------------------------------------------------------*/
void Log_Write_Ch ( uint8_t ch ) {

  if (b_file_ok) {                          /* file is open     */
    f_write(&file, &ch, 1, &ui_written);    /* write character  */
    if (1 != ui_written) {                  /* no file space    */
      f_close(&file);                       /* close file       */
      b_file_ok = FALSE;                    /* halt logging     */
    } else {                                /* write successfull */

    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   writes a string to SD card
 * @param   psz_str = pointer to string
 * @param   uc_len = string length
 * @return  -
 * @remarks limits number of characters to 200,
 *          closes log file if there is no file space
 *
 *----------------------------------------------------------------------------*/
void Log_Write_Str ( uint8_t * psz_str, uint8_t uc_len ) {

  if (uc_len > 200) { uc_len = 200; }

  if (b_file_ok) {                                  /* file is open     */
    f_write(&file, psz_str, uc_len, &ui_written);   /* write string     */
    if (uc_len != ui_written) {                     /* no file space    */
      f_close(&file);                               /* close file       */
      b_file_ok = FALSE;                            /* halt logging     */
    } else {                                        /* write successfull */

    }
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   float to ASCII conversion
 * @param   f_val = value to be converted
 * @param   p_string = pointer to destination string
 * @return  length of resulting string
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
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
