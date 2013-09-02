/**
******************************************************************************
* @file l3g_i2c.c
* @author Peter Lanius
* @version V0.10
* @date 09/12/2009
* @brief This file provides functions to operate the L3GV02DQ 3-axis
         accelerometer in I2C bus mode.
******************************************************************************
* @copy
*/

/* Includes ----------------------------------------------------------*/

#include "l3g_i2c.h"
#include "stm32f10x_i2c. h"

/* Private typedef ----------------------------------------------------------*/
/* Private define ----------------------------------------------------------*/

#define I2C_Speed 100000 // 100kHz bus speed (up to 400kHz is ok)
#define I2C_SLAVE_ADDRESS7 0xA0 // I2C own address if in slave mode

/* Private macro ----------------------------------------------------------*/
/* Private variables ----------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------------------*/

void GPIO_Configuration( void);
void I2C_Configuration( void);

/* Private functions ------------ --------- --------- --------- --------- --------- */

/**
* @brief Configure the used I/O ports pin
* @param None
* @retval None
*/
void GPIO_Configuration( void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure I2C_L3G pins: SCL and SDA */
    GPIO_InitStructure. GPIO_Pin = I2C_L3G_SCL | I2C_L3G_SDA;
    GPIO_InitStructure. GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure. GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_L3G_GPIO, &GPIO_InitStructure );
}

/**
* @brief I2C Configuration
* @param None
* @retval None
*/
void I2C_Configuration( void)
{
    I2C_InitTypeDef I2C_InitStructure;

    /* I2C configuration */
    I2C_InitStructure. I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure. I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure. I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure. I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure. I2C_Acknowledged Address = I2C_AcknowledgedAdd ress_7bit;
    I2C_InitStructure. I2C_ClockSpeed = I2C_Speed;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_L3G, ENABLE);

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_L3G, &I2C_InitStructure) ;
}

/**
* @brief Initializes peripherals used by the I2C L3G driver.
* @param None
* @retval None
*/
#define TxBuffer1Size (countof(TxBuffer1) - 1)
#define TxBuffer2Size (countof(TxBuffer2) - 1)

/* Private macro ----------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)) )

void I2C_L3G_Init( )
{
    uint8_t TxBuffer1[] = "\n\rL3G4200D Initialisation OK.\n\r";
    uint8_t TxBuffer2[] = "\n\rL3G4200D Initialisation FAILED.\n\r" ;
    uint8_t TxCounter = 0;
    uint8_t l3g_addr = 0;

    /* I2C Periph clock enable */
    RCC_APB1PeriphClock Cmd(I2C_L3G_CLK, ENABLE);

    /* GPIO Periph clock enable */
    RCC_APB2PeriphClock Cmd(I2C_L3G_GPIO_CLK, ENABLE);

    /* GPIO configuration */
    GPIO_Configuration( );

    /* I2C configuration */
    I2C_Configuration( );

    /* Initialise the device as per data sheet */
    I2C_L3G_Write(L3G_CTRL_REG1, L3G_INIT_CODE) ;
    I2C_L3G_Write(L3G_CTRL_REG2, 0x00);

    /* check everything is ok by reading whoami register */
    l3g_addr = I2C_L3G_ReadByte( L3G_WHO_AM_I);

    if (l3g_addr == L3G_SLAVE_ADDR) { /* Match - Send the relevant message */
        while (TxCounter+ + < TxBuffer1Size)
        {
            USART_SendData( USART1, TxBuffer1[TxCounter - 1]);
            while(USART_GetFlagStatus( USART1, USART_FLAG_TXE) == RESET);
        }
    } else {        /* Mismatch - Send the relevant message */
        while (TxCounter+ + < TxBuffer2Size)
        {
            USART_SendData( USART1, TxBuffer2[TxCounter - 1]);
            while(USART_GetFlagStatus( USART1, USART_FLAG_TXE) == RESET);
        }
    }

    /* Set the data rate to the desired rate */
    I2C_L3G_Write(L3G_CTRL_REG1, L3G_RUN_CODE) ;
}

/**
* @brief Write a byte to the specified register of the L3G sensor
* @param WriteAddr : 8bit write address of the L3G register
* @param DataByte : byte to write to the specified register
* @retval none
*/
void I2C_L3G_Write( uint8_t WriteAddr, uint8_t DataByte)
{
    I2C_GenerateSTART(I2C_L3G, ENABLE);                                /* Send START condition */
    while (!I2C_CheckEvent(I2C_L3G, I2C_EVENT_MASTER_MODE_SELECT) );   /* Test on EV5 and clear it */

    I2C_Send7bitAddress(I2C_L3G, L3G_SLAVE_ADDR, I2C_Direction_Transmitter); /* Send L3G address for write */
    while (!I2C_CheckEvent(I2C_L3G, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) );    /* Test on EV6 and clear it */

    I2C_SendData(I2C_L3G, WriteAddr);                                  /* Send the L3G's internal register address to write to */
    while (!I2C_CheckEvent(I2C_L3G, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));    /* Test on EV8 and clear it */

    I2C_SendData(I2C_L3G, DataByte);                                   /* Send the byte to be written */
    while (!I2C_CheckEvent(I2C_L3G, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));    /* Test on EV8 and clear it */

    I2C_GenerateSTOP(I2C_L3G, ENABLE);                                 /* Send STOP condition */
}

