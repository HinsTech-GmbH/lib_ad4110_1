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
#include <event_groups.h>

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

    // event group used for handling interrupts and state changes internally
    StaticEventGroup_t event_group_buffer;
    EventGroupHandle_t event_group;

    // enumeration indicating what the interrupt handlers should do. This is set by
    // the stream processor or the single data read function to configure the interrupts 
    // for the required purpose
    enum class int_func_t
    {
        IDLE,           // the controller is idle, no data reading active in the background, interrupts do  nothing
        SINGLE_READ,    // single read in progress in the background, interrupts need to handle that and then stop
        STREAM_READ,    // continuous stream read in progress, interrupts need to continuously refresh the stream read
    } int_func;

    // a flag indicating that continuous steam reading is enabled.
    bool stream_enabled = false;
    // a continuous stream reading is currently paused for the program to access the SPI interface. While this flag is set,
    // the stream might still be enabled (which is defined by the stream_enabled flag). If the stream is still enabled
    // and is paused, the part of the program that paused the stream to temporarily access the chip must resume it after
    // is done with the access. This is handled by giveChipAccess and the ADScopedAccess helper class.
    bool stream_paused = false;
    // a flag set by getChipAccess to tell the stream processor to pause continuous stream reading and temporarily give up access.
    bool stream_pause_enquiry = false;

    // status code variable set in interrupt handler during single read initiation so 
    // the blocking code waiting for the interrupt can read the result.
    el::retcode single_read_status = el::retcode::ok;

    // semaphore go guard device access (especially in combination with non-blocking background data streaming)
    StaticSemaphore_t sem_buffer_device_guard;
    SemaphoreHandle_t sem_device_guard = nullptr;

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

    /**
     * @brief same as xmitBytes but non-blocking. It uses HAL SPI DMA functions
     * to start an SPI Transmit+Receive operation with _size bytes and returns immediately.
     * This does not use any cpu until the entire SPI frame has been transmitted/received.
     * 
     * @param _size 
     * @return el::retcode 
     * @retval ok - started successfully
     * @retval busy - SPI already busy
     * @retval err - some other SPI error
     */
    el::retcode xmitBytesDMA(uint8_t _size);

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

    /**
     * @brief pauses any possible background data fetching
     * the chip and makes sure no other task apart from the calling one
     * can access the chip until giveChipAccess is called using a semaphore.
     * 
     * @return el::retcode
     * @retval ok - access acquired
     * @retval nolock - timeout while taking semaphore
     */
    el::retcode getChipAccess();
    
    /**
     * @brief gives access to the chip await from the calling task
     * allowing others to access it and resumes any background data
     * fetching if it was paused by getChipAccess() before.
     */
    void giveChipAccess();

    // friend declaration so ADScopedAccess can call the above two methods
    friend class ADScopedAccess;


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
     * @retval see readRegister()
     */
    el::retcode loadAllRegisters();

    /**
     * @brief writes every writable register from the local cache to the chip.
     * This can be used to change a bunch of values at once and then quickly 
     * and easily flush them to the chip.
     * This method aborts as soon as one write fails
     * 
     * @return el::retcode 
     * @retval see writeRegister();
     */
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

    /**
     * @brief reads the status registers from the chip and aborts
     * as soon as one read fails.
     * 
     * @return el::retcode 
     * @retval see readRegister()
     */
    el::retcode updateStatus();

    /**
     * @brief waits for the next ADC sample to become ready for reading and 
     * reads it. The waiting task is performed CPU-saving using interrupts
     * and FreeRTOS event groups. Other tasks can run while waiting for 
     * the ADC to become ready for read.
     * 
     * @param _output [out] data received from ADC.
     * @return el::retcode 
     * @retval see readRegister()
     * @retval invalid - can also mean invalid interrupt configuration (aka. int_func wasn't set to IDLE before getting access which should always happen)
     * @retval timeout - can also mean that ready signal was never received
     */
    el::retcode readData(uint32_t *_output);

    /**
     * @brief method to be called in the ISR of the ready pin
     * (when the ready pin goes low)
     */
    void readyInterruptHandler();

    /**
     * @brief method to be called in the SPI TxRx
     * complete ISR (or the callback) which is fired
     * whenever IT or DMA TransmitReceive operation is done.
     */
    void xmitCompleteHandler();

};


class ADScopedAccess
{
    AD4110 *ad;

public:

    el::retcode status;
    
    ADScopedAccess(AD4110 *_ad)
        : ad(_ad)
    {
        status = ad->getChipAccess();
    }

    ~ADScopedAccess()
    {
        ad->giveChipAccess();
    }
    
};