/**
 * @file ad4110.hpp
 * @author melektron
 * @brief
 * @version 0.1
 * @date 2023-07-20
 *
 * @copyright Copyright HinsTech GmbH (c) 2023
 *
 */

#pragma once

#include <stdint.h>
#include <uc_gpio.hpp>
#include <el/retcode.hpp>
#include <FreeRTOS.h>
#include <semphr.h>


#define AD_READ         0b01000000
#define AD_WRITE        0b00000000
#define AD_ADDR(addr)   ((addr) << 4)

// Registers: MSB specifies AFE(1)/ADC(0), 4 LSBs register address
// AFE registers
#define AD_AFE_REG_AFE_TOP_STATUS       0x80
#define AD_AFE_REG_AFE_CNTRL1           0x81
// reserved 0x82
#define AD_AFE_REG_AFE_CLK_CTRL         0x83
#define AD_AFE_REG_AFE_CNTRL2           0x84
#define AD_AFE_REG_PGA_RTD_CTRL         0x85
#define AD_AFE_REG_AFE_ERR_DISABLE      0x86
#define AD_AFE_REG_AFE_DETAIL_STATUS    0x87
// reserved 0x88
// reserved 0x89
// reserved 0x8a
// reserved 0x8b
#define AD_AFE_REG_AFE_CAL_DATA         0x8c
#define AD_AFE_REG_AFE_RSENSE_DATA      0x8d
#define AD_AFE_REG_NO_PWR_DEFAULT_SEL   0x8e
#define AD_AFE_REG_NO_PWR_DEFAULT_STATUS    0x8f

// ADC registers
#define AD_ADC_REG_ADC_STATUS           0x00
#define AD_ADC_REG_ADC_MODE             0x01
#define AD_ADC_REG_ADC_INTERFACE        0x02
#define AD_ADC_REG_ADC_CONFIG           0x03
#define AD_ADC_REG_DATA                 0x04
#define AD_ADC_REG_FILTER               0x05
#define AD_ADC_REG_ADC_GPIO_CONFIG      0x06
#define AD_ADC_REG_ID                   0x07
#define AD_ADC_REG_ADC_OFFSET0          0x08
#define AD_ADC_REG_ADC_OFFSET1          0x09
#define AD_ADC_REG_ADC_OFFSET2          0x0a
#define AD_ADC_REG_ADC_OFFSET3          0x0b
#define AD_ADC_REG_ADC_GAIN0            0x0c
#define AD_ADC_REG_ADC_GAIN1            0x0d
#define AD_ADC_REG_ADC_GAIN2            0x0e
#define AD_ADC_REG_ADC_GAIN3            0x0f


// general defines
// SPI R/W buffer size [bytes]
#define AD_SPI_BUFFER_SIZE 8

// register size enums used for general register reading and writing.
// this enumerates to the size int bytes.
enum ad_reg_size_t
{
    AD_REG_8 = 1,
    AD_REG_16 = 2,
    AD_REG_24 = 3,
};

class AD4110
{
private:    // data
    // The SPI peripheral the AD4110-1 chip is connected to
    SPI_HandleTypeDef &hspi;
    // The address of the AD4110-1 chip on the bus (this chip has a special way of using addresses over SPI)
    uint8_t address;
    // The CS pin the AD4110-1 is connected to (can be shared with multiple AD4110-1 if they have different pins)
    uc::Pin &cs_pin;

    // SPI communication buffers
    uint8_t spi_write_buffer[AD_SPI_BUFFER_SIZE];
    uint8_t spi_read_buffer[AD_SPI_BUFFER_SIZE];

    // semaphore to guard SPI peripheral
    StaticSemaphore_t sem_buffer_spi_guard;
    SemaphoreHandle_t sem_spi_guard = nullptr;

    // global timeout for any operation
    TickType_t timeout_ticks;

private:    // methods

    /**
     * @brief transmits _size bytes from the SPI write buffer. This function handles
     * the access semaphore to avoid race conditions. If the semaphore is taken,
     * this function will block for the selected timeout value and return nolock on timeout.
     * 
     * @param _size number of bytes to be transmitted
     * 
     * @return el::retcode 
     * @retval ok - write successful
     * @retval busy - SPI already busy
     * @retval err - some other SPI error
     * @retval nolock - SPI guard semaphore timeout
     */
    el::retcode transmitBytes(uint8_t _size);

    /**
     * @brief transmits _size bytes from the SPI write buffer and receives the same
     * amount of bytes in the SPI read buffer. This function handles
     * the access semaphore to avoid race conditions. If the semaphore is taken,
     * this function will block for the selected timeout value and return nolock on timeout.
     * 
     * @param _size number of bytes to be xmitted
     * 
     * @return el::retcode 
     * @retval ok - read/write successful
     * @retval busy - SPI already busy
     * @retval err - some other SPI error
     * @retval nolock - SPI guard semaphore timeout
     */
    el::retcode xmitBytes(uint8_t _size);



public:

    /**
     * @brief Construct a new AD4110 driver instance
     *
     * @param _hspi The SPI peripheral the AD4110-1 chip is connected to
     * @param _addr The address of the AD4110-1 chip on the bus (this chip has a special way of using addresses over SPI)
     * @param _cs_pin The CS pin the AD4110-1 is connected to (can be shared with multiple AD4110-1 if they have different pins)
     * @param _timeout The timeout used for all operations needing to take peripheral semaphores
     */
    AD4110(
        SPI_HandleTypeDef &_hspi,
        uint8_t _addr,
        uc::Pin &_cs_pin,
        TickType_t _timeout = 1000
    );

    /**
     * @brief resets the AD4110-1 chip by writing 64 1s to it.
     * 
     * @return el::retcode 
     * @retval ok - write successful
     * @retval busy - SPI already busy
     * @retval err - some other SPI error or invalid parameter
     * @retval nolock - SPI guard semaphore timeout
     */
    el::retcode reset();

    /**
     * @brief writes to a register of the AD4110-1. It simply sends the write command and then
     * the data.
     * 
     * @param _reg register definition
     * @param _size size of data to be written. use enumerations.
     * @param _data data to be written. Only the amount of bytes defined by the data size is
     * written, right aligned (size 8 writes the lowest byte, 16 the two lowest bytes and 24
     * the three lowest bytes. The highest byte is always ignored.)
     * 
     * @return el::retcode 
     * @retval ok - write successful
     * @retval busy - SPI already busy
     * @retval err - some other SPI error or invalid parameter
     * @retval nolock - SPI guard semaphore timeout
     * @retval invalid - invalid size
     */
    el::retcode writeRegister(uint8_t _reg, ad_reg_size_t _size, uint32_t _data);

    /**
     * @brief reads to a register from the AD4110-1. It simply sends the read command and then
     * reads
     * 
     * @param _reg register definition
     * @param _size size of data to be written. use enumerations.
     * @param _data [out] data to be written. Only the amount of bytes defined by the data size is
     * written, right aligned (size 8 writes the lowest byte, 16 the two lowest bytes and 24
     * the three lowest bytes. The highest byte is always ignored.)
     * 
     * @return el::retcode 
     * @retval ok - write successful
     * @retval busy - SPI already busy
     * @retval err - some other SPI error or invalid parameter
     * @retval nolock - SPI guard semaphore timeout
     * @retval invalid - invalid size
     */
    el::retcode readRegister(uint8_t _reg, ad_reg_size_t _size, uint32_t *_data);
};