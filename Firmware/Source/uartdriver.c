//============================================================================+
//
// $RCSfile: uartdriver.c,v $ (SOURCE FILE)
// $Revision: 1.7 $
// $Date: 2009/10/31 15:12:58 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             uart driver
//
//  CHANGES     UART 0 configured with 7 bit data length
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
    // Enable I/O port A, D.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 

    //
    // Set GPIO A0, A1, D2, D3 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Configure UART 0 for 115,200, 7-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_7 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Configure UART 1 for 115,200, 7-N-1 operation.
    //
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_7 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

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
/// \REMARKS     
///              
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
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
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
