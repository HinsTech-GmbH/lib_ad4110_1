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
    // a flag set by getChipAccess to tell the stream processor to stop continuous stream reading and give semaphore access away.
    bool stream_stop_enquiry = false;
    // enumeration set by interrupts in stream mode so that the stream stop function can
    // detect in what phase of the transfer the stream processor is and stop it propery
    enum class frame_phase_t
    {
        AWAITING_READY,
        DMA_IN_PROGRESS
    } frame_phase;

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
     * Initialization functions.
     */

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
     * Configuration functions.
     * 
     * These don't read data from the AD4110-1 but
     * rather configure it.
     */

    /**
     * @brief enumeration used to specify whether a part
     * (such as a current sense resistor) is internal
     * or external
     */
    enum int_ext_t
    {
        INTERNAL,
        EXTERNAL
    };

    /**
     * @brief selects between an internal and external current 
     * sense resistor.
     * 
     * @param _resistor INTERNAL or EXTERNAL
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid selection
     */
    el::retcode selectRSense(int_ext_t _resistor);

    /**
     * @brief selects between an internal and external RTD
     * reference resistor
     * 
     * @param _resistor INTERNAL or EXTERNAL
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid selection
     */
    el::retcode selectRRTD(int_ext_t _resistor);

    /**
     * @brief selects between an internal and external 
     * 2.5V voltage reference
     * 
     * @param _reference INTERNAL or EXTERNAL
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid selection
     */
    el::retcode selectVoltageReference(int_ext_t _reference);

    /**
     * @brief enumeration to specify the ADC channel
     */
    enum channel_t
    {
        CH0_HV = 0,
        CH1_LV1 = 1,
        CH2_LV2 = 2,
        CH3_LVDIFF = 3
    };

    /**
     * @brief enables channel _ch
     * 
     * @param _ch channel 
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid channel
     */
    el::retcode enableChannel(channel_t _ch);

    /**
     * @brief disables channel _ch
     * 
     * @param _ch channel 
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid channel
     */
    el::retcode disableChannel(channel_t _ch);

    enum sample_rate_t
    {
        SR_125K_SPS = 0,
        SR_62K5_SPS,
        SR_31K2_SPS,
        SR_25K_SPS,
        SR_15K6_SPS,
        SR_10K4_SPS,
        SR_5K_SPS,
        SR_2K5_SPS,
        SR_1K_SPS,
        SR_500_SPS,
        SR_400_SPS,
        SR_200_SPS,
        SR_100_SPS,
        SR_60_SPS,
        SR_50_SPS,
        SR_20_SPS,
        SR_16_SPS,
        SR_10_SPS,
        SR_5_SPS
    };

    /**
     * @brief set the sample rate.
     * 
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid sample rate
     */
    el::retcode setSampleRate(sample_rate_t _sr);

    /**
     * @brief enumeration for all the modes the input can be configured for
     */
    enum input_mode_t
    {
        IM_VOLTAGE, // Voltage measurement (+-10V)
        IM_CURRENT, // Current measurement (+-20mA)
        IM_RTD2W,   // 2 Wire resistive measurement
        IM_RTD3W,   // 3 Wire resistive measurement
        IM_RTD4W,   // 4 Wire resistive measurement
        IM_THERMO   // Junction thermocouple measurement
    };

    /**
     * @brief configures the AD4110-1 for the desired
     * measurement. This feature selects the most commonly
     * used configuration of additional features such as bias voltage
     * and pull up/down currents.
     * This function may need to configure multiple registers.
     * As soon as one write fails, it returns the error
     * 
     * @param _input_mode the measurement mode (aka. what you want to measure)
     * @return el::retcode 
     * @retval see writeRegister()
     * @retval invalid - can also mean invalid input mode
     */
    el::retcode setInputMode(input_mode_t _input_mode);

    /**
     * @brief turns the bias voltage on/off
     * 
     * @param _on true = on, false = off
     * @return el::retcode 
     * @retval see writeRegister()
     */
    el::retcode setBiasVoltage(bool _on);

    /**
     * @brief enumeration for all the gain configurations which enumerate to the bits
     */
    enum gain_t
    {
        GAIN_0p2 = AD_BITS_GAIN_0p2,
        GAIN_0p25 = AD_BITS_GAIN_0p25,
        GAIN_0p3 = AD_BITS_GAIN_0p3,
        GAIN_0p375 = AD_BITS_GAIN_0p375,
        GAIN_0p5 = AD_BITS_GAIN_0p5,
        GAIN_0p75 = AD_BITS_GAIN_0p75,
        GAIN_1 = AD_BITS_GAIN_1,
        GAIN_1p5 = AD_BITS_GAIN_1p5,
        GAIN_2 = AD_BITS_GAIN_2,
        GAIN_3 = AD_BITS_GAIN_3,
        GAIN_4 = AD_BITS_GAIN_4,
        GAIN_6 = AD_BITS_GAIN_6,
        GAIN_8 = AD_BITS_GAIN_8,
        GAIN_12 = AD_BITS_GAIN_12,
        GAIN_16 = AD_BITS_GAIN_16,
        GAIN_24 = AD_BITS_GAIN_24,
    };

    /**
     * @brief sets the input gain configuration. By default, gain
     * is automatically set to 0.2 in all voltage modes (V, RTD, Thermo)
     * and to 4 in current mode. Every time the input mode is changed,
     * gain is reset to these defaults. If a different gain is required,
     * it has to be set every time after changing the input mode.
     * 
     * You might want to increase the gain in Thermo and RTD modes because
     * of the small voltages involved.
     * 
     * Caution: Setting the gain too high may damage the internal 
     * circuitry of the AFE, so be careful. The gain should almost never 
     * be changed in direct voltage mode
     * 
     * @param _gain the new gain to set (use enumeration type AD4110::gain_t, not AD_BITS_GAIN_... definitions)
     * @return el::retcode 
     * @retval see writeRegister()
     */
    el::retcode setGain(gain_t _gain);


    /**
     * Information read operations.
     * Data streaming, reading status.
     */

    /**
     * @return uint32_t latest raw value read from chip that
     * is stored in the register cache
     */
    uint32_t getLatestRawValue();

    /**
     * @brief reads all the status registers from the chip and aborts
     * as soon as one read fails.
     * 
     * @return el::retcode 
     * @retval see readRegister()
     */
    el::retcode updateStatus();

    /**
     * @brief starts automatic streaming of ADC data in the background
     * using DMA and interrupts
     * 
     * @return el::retcode
     * @retval ok - stream started
     * @retval nak - stream was already running
     * @retval nolock - couldn't get access to device in time
     */
    el::retcode startStream();
    
    /**
     * @brief stops the automatic streaming of ADC data in the background.
     * 
     * @return el::retcode 
     * @retval ok - stream stopped
     * @retval nak - stream wasn't running
     */
    el::retcode stopStream();

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
     * External infrastructure.
     * Event handlers that need to be called from an external
     * ISR.
     */

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