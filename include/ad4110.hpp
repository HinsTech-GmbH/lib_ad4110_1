/**
 * @file ad4110.hpp
 * @author melektron
 * @brief
 * @version 0.1
 * @date 2023-07-20
 *
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 *
 */

#pragma once

#include <stdint.h>
#include <uc_gpio.hpp>
#include <el/retcode.hpp>
#include <FreeRTOS.h>
#include <semphr.h>

#include "ad4110_regs.hpp"


#define AD_READ         0b01000000
#define AD_WRITE        0b00000000
#define AD_ADDR(addr)   ((addr) << 4)

// general defines
// SPI R/W buffer size [bytes]
#define AD_SPI_BUFFER_SIZE 8


class AD4110
{
private:    // data
    // The SPI peripheral the AD4110-1 chip is connected to
    SPI_HandleTypeDef &hspi;
    // The address of the AD4110-1 chip on the bus (this chip has a special way of using addresses over SPI)
    uint8_t address;
    // The CS pin the AD4110-1 is connected to (can be shared with multiple AD4110-1 if they have different pins)
    uc::Pin &cs_pin;
    // The !READY pin of the AD4110-1 used for reading ADC data correctly
    uc::Pin &ready_pin;

    // SPI communication buffers
    uint8_t spi_write_buffer[AD_SPI_BUFFER_SIZE];
    uint8_t spi_read_buffer[AD_SPI_BUFFER_SIZE];

    // semaphore to guard SPI peripheral
    StaticSemaphore_t sem_buffer_spi_guard;
    SemaphoreHandle_t sem_spi_guard = nullptr;

    // global timeout for any operation
    TickType_t timeout_ticks;

public:     // data
    ad_regs_afe_t afe_regs;
    ad_regs_adc_t adc_regs;

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
        uc::Pin &_ready_pin,
        TickType_t _timeout = 1000
    );

    /**
     * @brief initializes complex features of the class
     * such as FreeRTOS semaphores. They cannot be initialized in the
     * constructor because it causes some wired problems.
     */
    void initialize();

    /**
     * @brief resets the AD4110-1 chip by writing 64 1s to it.
     * 
     * @return el::retcode
     * @retval see transmitBytes
     */
    el::retcode reset();

    /**
     * @brief reads every register and updates the local cache with
     * the values. This method aborts as soon as one read operation fails
     * and returns the error code.
     * 
     * @return el::retcode 
     * @retval see writeRegister()
     */
    el::retcode loadAllRegisters();

    el::retcode writeAllRegisters();

    /**
     * @brief configures the chip with the minimum changes that need to be made 
     * for it to run. For certain settings, default values will be used that can 
     * be changed later. If any write operation fails, this function returns
     * immediately. In order for it to be able to properly function, the register
     * cache needs to have been initialized using loadAllRegisters() beforehand.
     * 
     * @return el::retcode 
     * @retval see writeRegister()
     */
    el::retcode setup();

    el::retcode updateStatus();

    el::retcode readData(uint32_t *_output);


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