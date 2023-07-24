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

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "ad4110.hpp"

AD4110::AD4110(
    SPI_HandleTypeDef &_hspi,
    uint8_t _addr,
    uc::Pin &_cs_pin,
    TickType_t _timeout
) :
    hspi(_hspi),
    address(_addr),
    cs_pin(_cs_pin),
    timeout_ticks(_timeout)
{
    sem_spi_guard = xSemaphoreCreateBinaryStatic(&sem_buffer_spi_guard);
    configASSERT(sem_spi_guard);
}

el::retcode AD4110::transmitBytes(uint8_t _size)
{
    el::retcode retval = el::retcode::ok;
    HAL_StatusTypeDef status;

    if (!xSemaphoreTake(sem_spi_guard, timeout_ticks))
        return el::retcode::nolock;

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

cleanup:
    xSemaphoreGive(sem_spi_guard);
    return retval;
}

el::retcode AD4110::xmitBytes(uint8_t _size)
{
    el::retcode retval = el::retcode::ok;
    HAL_StatusTypeDef status;

    if (!xSemaphoreTake(sem_spi_guard, timeout_ticks))
        return el::retcode::nolock;

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

cleanup:
    xSemaphoreGive(sem_spi_guard);
    return retval;
}

el::retcode AD4110::writeRegister(uint8_t _reg, ad_reg_size_t _size, uint32_t _data)
{
    // prepare data
    uint8_t data_len = _size + 1;   // +1 byte for command
    spi_write_buffer[0] = AD_WRITE | AD_ADDR(address) | _reg;
    if (_size == AD_REG_8)
    {
        spi_write_buffer[1] = (uint8_t)(_data & 0xff);
    }
    else if (_size == AD_REG_16)
    {
        spi_write_buffer[1] = (uint8_t)(_data >> 1 & 0xff);
        spi_write_buffer[2] = (uint8_t)(_data & 0xff);
    }
    else if (_size == AD_REG_24)
    {
        spi_write_buffer[1] = (uint8_t)(_data >> 2 & 0xff);
        spi_write_buffer[2] = (uint8_t)(_data >> 1 & 0xff);
        spi_write_buffer[3] = (uint8_t)(_data & 0xff);
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
    else
        return el::retcode::invalid;

    return retval;
}

el::retcode AD4110::reset()
{
    uint8_t data_len = 8;
    memset(spi_write_buffer, 0xff, data_len);

    return transmitBytes(data_len);
}