/**
 * @file ad4110_read.cpp
 * @author melektron (matteo@elektron.work)
 * @brief data stream and general data read functions for AD4110-1 driver class
 * @version 0.1
 * @date 2023-07-27
 * 
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 * 
 */

#include "ad4110.hpp"


#define AD_EVG_IT_READY         (0b1 << 0)
#define AD_EVG_IT_READ_DONE     (0b1 << 1)
#define AD_EVG_IT_READ_FAIL     (0b1 << 2)


el::retcode AD4110::updateStatus()
{   
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);

    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_DETAIL_STATUS, AD_AFE_REG_SIZE_AFE_DETAIL_STATUS, &afe_regs.afe_detail_status));
    
    return el::retcode::ok;
}

uint32_t AD4110::getLatestValueRaw()
{
    return adc_regs.data;
}

float AD4110::getLatestValue()
{
    return (adc_regs.data - 0x800000) * data_scaling_factor;
}

float AD4110::getMaximumInputValue()
{
    return max_input_value;
}

el::retcode AD4110::startStream()
{
    // stream already running
    if (stream_enabled) return el::retcode::nak;

    // wait for everything else accessing the chip to finish and then get access
    if (!xSemaphoreTake(sem_device_guard, timeout_ticks))
        return el::retcode::nolock;

    stream_enabled = true;
    
    // configure interrupts for stream read
    int_func = int_func_t::STREAM_READ;

    // initiate the first read
    frame_phase = frame_phase_t::AWAITING_READY;
    ready_pin.clearPendingInterrupt();
    ready_pin.enableInterrupt();
    cs_pin.write(0);

    // after the read is done, a new one
    // will be initiated automatically

    return el::retcode::ok;
}

el::retcode AD4110::stopStream()
{
    // no stream running
    if (!stream_enabled) return el::retcode::nak;

    // stop any new ready interrupts from occurring
    ready_pin.disableInterrupt();

    // if we are in the phase of waiting for a ready interrupt
    // we can now just disable the chip and give up access from here
    if (frame_phase == frame_phase_t::AWAITING_READY)
    {
        cs_pin.write(1);
        xSemaphoreGive(sem_device_guard);
        // reset interrupt function to nothing
        int_func = int_func_t::IDLE;
    }
    // if we are in the phase of waiting for DMA data transfer to 
    // finish, we cannot just give up access as the DMA is still using SPI.
    // instead, we set the enquiry flag so the transfer complete interrupt will do it once
    // DMA is done and so that it won't start the next cycle.
    // Then, the next task which takes the semaphore might block a bit until DMA is done.
    else if (frame_phase == frame_phase_t::DMA_IN_PROGRESS)
    {
        stream_stop_enquiry = true;
        // int_func is also set to IDLE by the DMA ISR
    }

    stream_enabled = false;
    // also clear the paused flag, so in case the stream is stopped manually after being paused
    // it won't be resumed.
    // If getChipAccess calls stopStream it will set the paused flag right after the function call again
    stream_paused = false;

    return el::retcode::ok;
}

el::retcode AD4110::readData(uint32_t *_output)
{
    // get access
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);
    
    // if interrupts are not configured to idle, something has gone wrong
    if (int_func != int_func_t::IDLE)
        return el::retcode::invalid;

    // we now cofigure interrupts for single read mode
    int_func = int_func_t::SINGLE_READ;

    // enable chip and wait for ready signal to become low
    frame_phase = frame_phase_t::AWAITING_READY;
    ready_pin.clearPendingInterrupt();
    ready_pin.enableInterrupt();
    cs_pin.write(0);
    auto bits = xEventGroupWaitBits(
        event_group,
        AD_EVG_IT_READ_DONE | AD_EVG_IT_READ_FAIL,
        pdTRUE,
        pdFALSE,
        timeout_ticks
    );

    // on error (error while starting SPI DMA feature)
    if (bits & AD_EVG_IT_READ_FAIL)
    {
        cs_pin.write(1);
        int_func = int_func_t::IDLE;
        return single_read_status;
    }
    // timeout (likely bc chip is misconfigured)
    if (!bits)
    {
        cs_pin.write(1);
        int_func = int_func_t::IDLE;
        return el::retcode::timeout;
    }

    // read done signal received, disable interrupt function and return data
    cs_pin.write(1);    // should already be the done in SPI done handler but just to make sure
    *_output = adc_regs.data;
    int_func = int_func_t::IDLE;
    return el::retcode::ok;
}


/**
 * Interrupt and event handlers.
 * 
 */

void AD4110::readyInterruptHandler()
{
    // disable any further interrupts (needed for both stream and single reading)
    ready_pin.disableInterrupt();

    // shouldn't happen but just in case we don't do anything.
    if (int_func == int_func_t::IDLE) return;

    // start receiving data (both in single and in stream mode)

    // if DATA_STAT bit is set, the ADC_STATUS register is appended to the data register,
    // making it one byte longer
    if (AD_TEST_BITS(adc_regs.adc_interface, AD_MASK_DATA_STAT))
    {
        uint8_t data_len = (AD_ADC_REG_SIZE_DATA + AD_ADC_REG_SIZE_ADC_STATUS) + 1;   // +1 byte for command
        spi_write_buffer[0] = AD_READ | AD_ADDR(address) | AD_ADC_REG_ADDR_DATA;
        spi_write_buffer[1] = 0;
        spi_write_buffer[2] = 0;
        spi_write_buffer[3] = 0;
        spi_write_buffer[4] = 0;

        single_read_status = xmitBytesDMA(data_len);
        if (single_read_status != el::retcode::ok)
            goto error;
    }
    // Otherwise, the data register can just be read with normal size
    else
    {
        uint8_t data_len = AD_ADC_REG_SIZE_DATA + 1;   // +1 byte for command
        spi_write_buffer[0] = AD_READ | AD_ADDR(address) | AD_ADC_REG_ADDR_DATA;
        spi_write_buffer[1] = 0;
        spi_write_buffer[2] = 0;
        spi_write_buffer[3] = 0;

        single_read_status = xmitBytesDMA(data_len);
        if (single_read_status != el::retcode::ok)
            goto error;
    }

    // we set the frame phase to waiting for DMA (only 
    // necessary for stream mode but we just always do it)
    frame_phase = frame_phase_t::DMA_IN_PROGRESS;

    // regular exit
    return;

    // error exit
error:

    // in case we got an error in single read mode, we just notify the reading task
    if (int_func == int_func_t::SINGLE_READ)
    {
        BaseType_t higher_priority_task_woken, result;
        result = xEventGroupSetBitsFromISR(
            event_group,
            AD_EVG_IT_READ_FAIL,
            &higher_priority_task_woken
        );

        if (result != pdFAIL)
            portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    // in case we got an error in stream read mode, we just start waiting for the next frame for now
    // TODO: add error counting and error flag
    else if (int_func == int_func_t::STREAM_READ)
    {
        cs_pin.write(1);
        frame_phase = frame_phase_t::AWAITING_READY;
        ready_pin.clearPendingInterrupt();
        ready_pin.enableInterrupt();
        cs_pin.write(0);
    }
    
}

void AD4110::xmitCompleteHandler()
{
    BaseType_t higher_priority_task_woken, result;

    // end the frame and disable chip
    cs_pin.write(1);    
    
    // process the received data
    // if we have data stat enabled, also read the status byte
    if (AD_TEST_BITS(adc_regs.adc_interface, AD_MASK_DATA_STAT))
    {
        adc_regs.adc_status = spi_read_buffer[4];
    }
    // always read the data bytes
    adc_regs.data = spi_read_buffer[1] << 16 | spi_read_buffer[2] << 8 | spi_read_buffer[3];

    // in single read mode, we now just notify the initiating task
    if (int_func == int_func_t::SINGLE_READ)
    {
        // notify blocking task that we are done
        result = xEventGroupSetBitsFromISR(
            event_group,
            AD_EVG_IT_READ_DONE,
            &higher_priority_task_woken
        );

        if (result != pdFAIL)
            portYIELD_FROM_ISR(higher_priority_task_woken);

    }

    // in stream mode we need to check if stream should be ended and restart it otherwise
    else if (int_func == int_func_t::STREAM_READ)
    {
        // if we get a stop enquiry, we stop the stream and give access away.
        if (stream_stop_enquiry)
        {
            stream_stop_enquiry = false;

            result = xSemaphoreGiveFromISR(sem_device_guard, &higher_priority_task_woken);
            if (result != pdFAIL)
                portYIELD_FROM_ISR(higher_priority_task_woken);
            
            // reset interrupt function to idle
            int_func = int_func_t::IDLE;
            
            return;
        }

        // otherwise we start waiting for the next frame
        frame_phase = frame_phase_t::AWAITING_READY;
        ready_pin.clearPendingInterrupt();
        ready_pin.enableInterrupt();
        cs_pin.write(0);
    }
}