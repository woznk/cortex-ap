//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief    uart driver
///
/// \file
///
//  CHANGES Baud rate della UART 1 ridotto a 4800 per invio tramite radio
//
//============================================================================*/

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uartdriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC  static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char s_ucBuffWrite0;    //
VAR_STATIC unsigned char s_ucBuffRead0;     //
VAR_STATIC unsigned char s_pucBuffer0[256]; //
VAR_STATIC unsigned char s_ucBuffWrite1;    //
VAR_STATIC unsigned char s_ucBuffRead1;     //
VAR_STATIC unsigned char s_pucBuffer1[256]; //

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION UART initialization
/// \RETURN      -
/// \REMARKS     The first UART (connected to the FTDI virtual serial port on
///              the evaluation board) will be configured in 115,200 baud,
///              8-n-1 mode.
///
///----------------------------------------------------------------------------
void
UARTInit(void)
{
    //
    // Enable UART 0, 1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Enable GPIO port A, D for UART 0, 1.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Set GPIO A0, A1 as UART 0 pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Set GPIO D2, D3 as UART 1 pins.
    //
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Select pin's alternate function
    //
#if defined(PART_LM3S9B90)
    GPIOPinConfigure(GPIO_PD2_U1RX);
    GPIOPinConfigure(GPIO_PD3_U1TX);
#endif

    //
    // Configure UART 0 for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Configure UART 1 for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 4800,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable UART 0 FIFO.
    //
    UARTFIFOEnable(UART0_BASE);

    //
    // Enable UART 1 FIFO.
    //
    UARTFIFOEnable(UART1_BASE);

    //
    // Set UART 0 FIFO level for TX / RX interrupt.
    //
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);

    //
    // Set UART 1 FIFO level for TX / RX interrupt.
    //
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);

    //
    // Enable UART 0 interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Enable UART 1 interrupt.
    //
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Initialize receive buffer pointers
    //
    s_ucBuffRead1 = 0;
    s_ucBuffWrite1 = 0;

}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Send a string to the UART 0.
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
UART0Send(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Wait for some space in transmit FIFO.
        //
        while (UARTSpaceAvail(UART0_BASE) == false)
        {
        }

        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Send a string to the UART 1.
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
void
UART1Send(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Wait for some space in transmit FIFO.
        //
        while (UARTSpaceAvail(UART1_BASE) == false)
        {
        }

        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART1_BASE, *pucBuffer++);
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION The UART interrupt handler.
/// \RETURN      -
/// \REMARKS     Duration ~2 us with system clock = 50 MHz
///              Periodicity ~1 ms with 115200 baud continuous data
///
///
///----------------------------------------------------------------------------
void
UART0IntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while (UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it to the buffer
        //
        s_pucBuffer0[s_ucBuffWrite0] = UARTCharGetNonBlocking(UART0_BASE);
        s_ucBuffWrite0++;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   gps receive interrupt routine
///
/// \remarks -
///
//----------------------------------------------------------------------------
void
UART1IntHandler( void )
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while (UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it to the buffer
        //
        s_pucBuffer1[s_ucBuffWrite1] = UARTCharGetNonBlocking(UART1_BASE);
        s_ucBuffWrite1++;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   get a character from uart 1 buffer
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean
UART1GetChar ( char *ch )
{
   if (s_ucBuffWrite1 == s_ucBuffRead1)
   {                  // buffer empty
      return false;
   }
   else
   {
      *ch = s_pucBuffer1[s_ucBuffRead1];                   // Return data
      s_ucBuffRead1++; // Update buffer index
      return true;
   }
}

//----------------------------------------------------------------------------
//
/// \brief   get a character from uart 0 buffer
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean
UART0GetChar ( char *ch )
{
   if (s_ucBuffWrite0 == s_ucBuffRead0)
   {                  // buffer empty
      return false;
   }
   else
   {
      *ch = s_pucBuffer0[s_ucBuffRead0];                   // Return data
      s_ucBuffRead0++; // Update buffer index
      return true;
   }
}
