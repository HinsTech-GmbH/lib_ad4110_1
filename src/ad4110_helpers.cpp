/**
 * @file ad4110_helpers.hpp
 * @author melektron
 * @brief helper functions and general read/write/access features (mostly private)
 * @version 0.1
 * @date 2023-07-20
 *
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 *
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdio.h>

#include "ad4110.hpp"


el::retcode AD4110::transmitBytes(uint8_t _size)
{
    el::retcode retval = el::retcode::ok;
    HAL_StatusTypeDef status;

    // just to make sure, should not happen with semaphore
    if (hspi.State != HAL_SPI_STATE_READY)
    {
        retval = el::retcode::busy;
        goto cleanup;
    }

    // start write
    cs_pin.write(0);
    status = HAL_SPI_Transmit(&hspi, spi_write_buffer, _size, timeout_ticks);

    // check outcome (in case busy flag changed while preparing data or another error occurred)
    if (status == HAL_BUSY)
    {
        // don't reset pin because it is probably used by 
        // something else (should not happen normally anyway)
        retval = el::retcode::busy;
        goto cleanup;
    }
    else if (status != HAL_OK)
    {
        cs_pin.write(1);
        retval = el::retcode::err;
        goto cleanup;
    }
    
    // on success, also end transmission
    cs_pin.write(1);

cleanup:
    return retval;
}

el::retcode AD4110::xmitBytes(uint8_t _size)
{
    el::retcode retval = el::retcode::ok;
    HAL_StatusTypeDef status;

    // just to make sure, should not happen with semaphore
    if (hspi.State != HAL_SPI_STATE_READY)
    {
        retval = el::retcode::busy;
        goto cleanup;
    }

    // start transfer
    cs_pin.write(0);
    status = HAL_SPI_TransmitReceive(&hspi, spi_write_buffer, spi_read_buffer, _size, timeout_ticks);

    // check outcome (in case busy flag changed while preparing data or another error occurred)
    if (status == HAL_BUSY)
    {
        // don't reset pin because it is probably used by 
        // something else (should not happen normally anyway)
        retval = el::retcode::busy;
        goto cleanup;
    }
    else if (status != HAL_OK)
    {
        cs_pin.write(1);
        retval = el::retcode::err;
        goto cleanup;
    }
    
    // on success, also end transmission
    cs_pin.write(1);

cleanup:
    return retval;
}

el::retcode AD4110::xmitBytesDMA(uint8_t _size)
{
    el::retcode retval = el::retcode::ok;
    HAL_StatusTypeDef status;

    if (hspi.State != HAL_SPI_STATE_READY)
    {
        retval = el::retcode::busy;
        goto cleanup;
    }

    // start transfer
    cs_pin.write(0);
    status = HAL_SPI_TransmitReceive_DMA(&hspi, spi_write_buffer, spi_read_buffer, _size);

    // check outcome 
    if (status == HAL_BUSY)
    {
        // don't reset pin because it is probably used by 
        // something else (should not happen normally anyway)
        retval = el::retcode::busy;
        goto cleanup;
    }
    else if (status != HAL_OK)
    {
        cs_pin.write(1);
        retval = el::retcode::err;
        goto cleanup;
    }

cleanup:
    return retval;
}

el::retcode AD4110::writeRegister(uint8_t _reg, ad_reg_size_t _size, uint32_t _data)
{
    // prepare command and data
    uint8_t data_len = _size + 1;   // +1 byte for command
    spi_write_buffer[0] = AD_WRITE | AD_ADDR(address) | _reg;
    if (_size == AD_REG_8)
    {
        spi_write_buffer[1] = (uint8_t)(_data & 0xff);
    }
    else if (_size == AD_REG_16)
    {
        spi_write_buffer[1] = (uint8_t)(_data >> 8 & 0xff);
        spi_write_buffer[2] = (uint8_t)(_data & 0xff);
    }
    else if (_size == AD_REG_24)
    {
        spi_write_buffer[1] = (uint8_t)(_data >> 16 & 0xff);
        spi_write_buffer[2] = (uint8_t)(_data >> 8 & 0xff);
        spi_write_buffer[3] = (uint8_t)(_data & 0xff);
    }
    else if (_size == AD_REG_32)
    {
        spi_write_buffer[1] = (uint8_t)(_data >> 24 & 0xff);
        spi_write_buffer[2] = (uint8_t)(_data >> 16 & 0xff);
        spi_write_buffer[3] = (uint8_t)(_data >> 8 & 0xff);
        spi_write_buffer[4] = (uint8_t)(_data & 0xff);
    }
    else
        return el::retcode::invalid;

    return transmitBytes(data_len);
}

el::retcode AD4110::readRegister(uint8_t _reg, ad_reg_size_t _size, uint32_t *_data)
{
    // prepare command
    uint8_t data_len = _size + 1;   // +1 byte for command
    spi_write_buffer[0] = AD_READ | AD_ADDR(address) | _reg;
    spi_write_buffer[1] = 0;
    spi_write_buffer[2] = 0;
    spi_write_buffer[3] = 0;
    spi_write_buffer[4] = 0;

    el::retcode retval = xmitBytes(data_len);

    // process the data
    if (_size == AD_REG_8)
    {
        *_data = spi_read_buffer[1];
    }
    else if (_size == AD_REG_16)
    {
        *_data = spi_read_buffer[1] << 8 | spi_read_buffer[2];
    }
    else if (_size == AD_REG_24)
    {
        *_data = spi_read_buffer[1] << 16 | spi_read_buffer[2] << 8 | spi_read_buffer[3];
    }
    else if (_size == AD_REG_32)
    {
        *_data = spi_read_buffer[1] << 24 | spi_read_buffer[2] << 16 | spi_read_buffer[3] << 8 | spi_read_buffer[4];
    }
    else
        return el::retcode::invalid;

    return retval;
}

el::retcode AD4110::getChipAccess()
{
    if (stream_enabled)
    {
        // stop the stream (ignore the error if it is already stopped)
        stopStream();
        // set stop flag so we can remember to restart the stream in giveChipAccess()
        stream_paused = true;
    }

    // wait for everything to finish and then get access
    if (!xSemaphoreTake(sem_device_guard, timeout_ticks))
        return el::retcode::nolock;
    return el::retcode::ok;
}

void AD4110::giveChipAccess()
{
    xSemaphoreGive(sem_device_guard);
    // if the stream was paused, we resume it
    if (stream_paused)
    {
        startStream();
        stream_paused = false;
    }
}

// conversion from gain_t enum to the gain factors it represents
static float gain_factors[] = {
    0.2,
    0.25,
    0.3,
    0.375,
    0.5,
    0.75,
    1,
    1.5,
    2,
    3,
    4,
    6,
    8,
    12,
    16,
    24
};
void AD4110::calculateFactorsForVoltageWithGain(gain_t _gain)
{
    // by default, gain is 0.2. This would result in scaling_factor * 1.
    // if gain is increased for example to 0.4 (which is not possible but just for 
    // simplicity well use that value), the reported value will be twice as large
    // while applying the sme voltage. To do so, we must half the factor, so now
    // we get scaling_factor * 0.2/0.4 = scaling_factor * 0.5
    float scaling_coefficient =  (gain_factors[GAIN_0p2] / gain_factors[_gain]);

    data_scaling_factor = V_base_scaling_factor * scaling_coefficient;
    max_input_value = 10.0f * scaling_coefficient;  // base = 10mA
}
void AD4110::calculateFactorsForCurrentWithGain(gain_t _gain)
{
    // same as with current, just with different values
    float scaling_coefficient = (gain_factors[GAIN_4] / gain_factors[_gain]);
    
    data_scaling_factor = mA_base_scaling_factor * scaling_coefficient;
    max_input_value = 20.0f * scaling_coefficient;  // base = 20mA
}
