/**
******************************************************************************
* @file lis3l_i2c.c
* @author Peter Lanius
* @version V0.10
* @date 09/12/2009
* @brief This file provides functions to operate the LIS3LV02DQ 3-axis
         accelerometer in I2C bus mode.
******************************************************************************
* @copy
*/

/* Includes ----------------------------------------------------------*/

#include "lis3l_i2c.h"
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

    /* Configure I2C_LIS3L pins: SCL and SDA */
    GPIO_InitStructure. GPIO_Pin = I2C_LIS3L_SCL | I2C_LIS3L_SDA;
    GPIO_InitStructure. GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure. GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_LIS3L_GPIO, &GPIO_InitStructure );
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
    I2C_Cmd(I2C_LIS3L, ENABLE);

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_LIS3L, &I2C_InitStructure) ;
}

/**
* @brief Initializes peripherals used by the I2C LIS3L driver.
* @param None
* @retval None
*/
#define TxBuffer1Size (countof(TxBuffer1) - 1)
#define TxBuffer2Size (countof(TxBuffer2) - 1)

/* Private macro ----------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)) )

void I2C_LIS3L_Init( )
{
    uint8_t TxBuffer1[] = "\n\rLIS3LV02DQ Initialisation OK.\n\r";
    uint8_t TxBuffer2[] = "\n\rLIS3LV02DQ Initialisation FAILED.\n\r" ;
    uint8_t TxCounter = 0;
    uint8_t lis3l_addr = 0;

    /* I2C Periph clock enable */
    RCC_APB1PeriphClock Cmd(I2C_LIS3L_CLK, ENABLE);

    /* GPIO Periph clock enable */
    RCC_APB2PeriphClock Cmd(I2C_LIS3L_GPIO_CLK, ENABLE);

    /* GPIO configuration */
    GPIO_Configuration( );

    /* I2C configuration */
    I2C_Configuration( );

    /* Initialise the device as per data sheet */
    I2C_LIS3L_Write(LIS3L_CTRL_REG1, LIS3L_INIT_CODE) ;
    I2C_LIS3L_Write(LIS3L_CTRL_REG2, 0x00);

    /* check everything is ok by reading whoami register */
    lis3l_addr = I2C_LIS3L_ReadByte( LIS3L_WHO_AM_I);

    if (lis3l_addr == LIS3L_SLAVE_ADDR) { /* Match - Send the relevant message */
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
    I2C_LIS3L_Write( LIS3L_CTRL_REG1, LIS3L_RUN_CODE) ;
}

/**
* @brief Write a byte to the specified register of the LIS3L sensor
* @param WriteAddr : 8bit write address of the LIS3L register
* @param DataByte : byte to write to the specified register
* @retval none
*/
void I2C_LIS3L_Write( uint8_t WriteAddr, uint8_t DataByte)
{
    /* Send START condition */
    I2C_GenerateSTART( I2C_LIS3L, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEve nt(I2C_LIS3L, I2C_EVENT_MASTER_MODE_SELECT) );

    /* Send LIS3L address for write */
    I2C_Send7bitAddress (I2C_LIS3L, LIS3L_SLAVE_ADDR, I2C_Direction_Transmitter) ;

    /* Test on EV6 and clear it */
    while(!I2C_CheckEve nt(I2C_LIS3L, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) );

    /* Send the LIS3L's internal register address to write to */
    I2C_SendData( I2C_LIS3L, WriteAddr);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEve nt(I2C_LIS3L, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));

    /* Send the byte to be written */
    I2C_SendData( I2C_LIS3L, DataByte);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEve nt(I2C_LIS3L, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));

    /* Send STOP condition */
    I2C_GenerateSTOP( I2C_LIS3L, ENABLE);
}

