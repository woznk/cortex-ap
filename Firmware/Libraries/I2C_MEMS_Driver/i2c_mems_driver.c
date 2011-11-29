//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief I2C driver for MEMS sensors
/// Changes: function ReadBuff(): added wait for flag BTF before disabling ACK
//
//============================================================================*/

#include "stm32f10x_i2c.h"
#include "i2c_mems_driver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
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

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   ReadReg
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, register address
/// \param   *data, pointer to destination data
/// \remarks see
///          https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=13303&Source=/public/STe2ecommunities/Tags.aspx?tags=i2c
///          http://read.pudn.com/downloads124/sourcecode/embed/527821/7.1%20-%20I2C/Application/I2C.c__.htm
///          http://corvusm3.googlecode.com/svn-history/r251/CorvusM3_FC/CorvusM3_Firmware/trunk/i2c.c
///
///----------------------------------------------------------------------------
uint8_t ReadReg(uint8_t slave, uint8_t reg, uint8_t* data)
{
	/* Wait while the bus is busy */
	while (I2C_GetFlagStatus(I2C_MEMS, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_MEMS, ENABLE);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send address for read */
    I2C_Send7bitAddress(I2C_MEMS, slave, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the sensor register address to read from */
    I2C_SendData(I2C_MEMS, reg);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send START condition */
    I2C_GenerateSTART(I2C_MEMS, ENABLE);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send address for read */
    I2C_Send7bitAddress(I2C_MEMS, slave, I2C_Direction_Receiver);
    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* Only one byte to read */
    I2C_AcknowledgeConfig(I2C_MEMS, DISABLE);

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_MEMS, ENABLE);

    /* Test on EV7 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_RECEIVED));
    /* Receive the byte to be read */
    *data = I2C_ReceiveData(I2C_MEMS);

    return 1;
}


///----------------------------------------------------------------------------
///
/// \brief   ReadBuff
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, address of starting register
/// \param   *data, pointer to destination data
/// \param   length, number of registers to read
/// \remarks -
///
///----------------------------------------------------------------------------
uint8_t ReadBuff(uint8_t slave, uint8_t reg, uint8_t* data, uint8_t length)
{
	/* Wait while the bus is busy */
	while (I2C_GetFlagStatus(I2C_MEMS, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_MEMS, ENABLE);
    /* EV5: test and clear SB flag */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send slave address + WRITE */
    I2C_Send7bitAddress(I2C_MEMS, slave, I2C_Direction_Transmitter);
    /* EV6: test and clear ADD flag */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the sensor register address */
    I2C_SendData(I2C_MEMS, reg);
    /* EV8: test and clear TXE flag */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send START condition */
    I2C_GenerateSTART(I2C_MEMS, ENABLE);
    /* EV5: test and clear SB flag */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send slave address + READ */
    I2C_Send7bitAddress(I2C_MEMS, slave, I2C_Direction_Receiver);
    /* EV6: test and clear ADD flag */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while (length > 3) {
        /* EV7: test RXNE flag */
        while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_RECEIVED));

        /* Read byte from data register, clear RXNE flag */
        *data++ = I2C_ReceiveData(I2C_MEMS);

        /* Update number of bytes to be read */
        length--;
    }

    /* Test RXNE and BTF flags (wait reception of bytes N-2 and N-1) */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF));

    /* Disable ACK */
    I2C_AcknowledgeConfig(I2C_MEMS, DISABLE);

    /* Read byte N-2, start reception of byte N, clear BTF (RXNE not cleared) */
    *data++ = I2C_ReceiveData(I2C_MEMS);

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_MEMS, ENABLE);

    /* Read byte N-1, clear RXNE flag */
    *data++ = I2C_ReceiveData(I2C_MEMS);

    /* EV7: test RXNE flag (receive byte N) */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_RECEIVED));

    /* Read byte N */
    *data = I2C_ReceiveData(I2C_MEMS);

    return 1;
}

///----------------------------------------------------------------------------
///
/// \brief   WriteReg
/// \return  1
/// \param   slave, address of slave device
/// \param   reg, register address
/// \param   data, data to be written
/// \remarks see
///
///----------------------------------------------------------------------------
uint8_t WriteReg(uint8_t slave, uint8_t reg, uint8_t data)
{
    /* Send START condition */
    I2C_GenerateSTART(I2C_MEMS, ENABLE);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send address for write */
    I2C_Send7bitAddress(I2C_MEMS, slave, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the sensor register address to write to */
    I2C_SendData(I2C_MEMS, reg);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(I2C_MEMS, data);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_MEMS, ENABLE);
    return 1;
}


///----------------------------------------------------------------------------
///
/// \brief   Configure the I/O ports pin used for I2C
/// \return  -
/// \param   -
/// \remarks -
///
///----------------------------------------------------------------------------
static void GPIO_MEMS_Configuration( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure I2C_LIS3L pins: SCL and SDA */
    GPIO_InitStructure. GPIO_Pin = I2C_MEMS_SCL | I2C_MEMS_SDA;
    GPIO_InitStructure. GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure. GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_MEMS_GPIO, &GPIO_InitStructure );
}

///----------------------------------------------------------------------------
///
/// \brief   Configure I2C
/// \return  -
/// \param   -
/// \remarks -
///
///----------------------------------------------------------------------------
static void I2C_Configuration( void )
{
    I2C_InitTypeDef I2C_InitStructure;

    /* I2C structure initialization */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_MEMS_Speed;

    /* I2C peripheral enable */
    I2C_Cmd(I2C_MEMS, ENABLE);

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_MEMS, &I2C_InitStructure) ;
}

///----------------------------------------------------------------------------
///
/// \brief   Initializes peripherals used by the I2C driver.
/// \return  -
/// \param   -
/// \remarks -
///
///----------------------------------------------------------------------------
void I2C_MEMS_Init( void )
{
    /* I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(I2C_MEMS_CLK, ENABLE);

    /* GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(I2C_MEMS_GPIO_CLK, ENABLE);

    /* GPIO configuration */
    GPIO_MEMS_Configuration( );

    /* I2C configuration */
    I2C_Configuration( );
}
