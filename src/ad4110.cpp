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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdio.h>

#include "ad4110.hpp"


#define AD_EVG_IT_READY         (0b1 << 0)
#define AD_EVG_IT_SPI_TX_DONE   (0b1 << 1)
#define AD_EVG_IT_SPI_RX_DONE   (0b1 << 2)


AD4110::AD4110(
    SPI_HandleTypeDef &_hspi,
    uint8_t _addr,
    uc::Pin &_cs_pin,
    uc::Pin &_ready_pin,
    TickType_t _timeout
) :
    hspi(_hspi),
    address(_addr),
    cs_pin(_cs_pin),
    ready_pin(_ready_pin),
    timeout_ticks(_timeout)
{}

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
    
    // on success, also end transmission
    cs_pin.write(1);

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
    
    // on success, also end transmission
    cs_pin.write(1);

cleanup:
    xSemaphoreGive(sem_spi_guard);
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

void AD4110::initialize()
{
    sem_spi_guard = xSemaphoreCreateBinaryStatic(&sem_buffer_spi_guard);
    configASSERT(sem_spi_guard);
    xSemaphoreGive(sem_spi_guard);

    event_group = xEventGroupCreateStatic(&event_group_buffer);
    configASSERT(event_group);
}

el::retcode AD4110::reset()
{
    uint8_t data_len = 8;
    memset(spi_write_buffer, 0xff, data_len);

    return transmitBytes(data_len);
}

el::retcode AD4110::loadAllRegisters()
{
    // AFE registers
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_TOP_STATUS,         AD_AFE_REG_SIZE_AFE_TOP_STATUS,        &afe_regs.afe_top_status));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_CNTRL1,             AD_AFE_REG_SIZE_AFE_CNTRL1,            &afe_regs.afe_cntrl1));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_CLK_CTRL,           AD_AFE_REG_SIZE_AFE_CLK_CTRL,          &afe_regs.afe_clk_ctrl));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_CNTRL2,             AD_AFE_REG_SIZE_AFE_CNTRL2,            &afe_regs.afe_cntrl2));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL,           AD_AFE_REG_SIZE_PGA_RTD_CTRL,          &afe_regs.pga_rtd_ctrl));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_ERR_DISABLE,        AD_AFE_REG_SIZE_AFE_ERR_DISABLE,       &afe_regs.afe_err_disable));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_DETAIL_STATUS,      AD_AFE_REG_SIZE_AFE_DETAIL_STATUS,     &afe_regs.afe_detail_status));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_CAL_DATA,           AD_AFE_REG_SIZE_AFE_CAL_DATA,          &afe_regs.afe_cal_data));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_AFE_RSENSE_DATA,        AD_AFE_REG_SIZE_AFE_RSENSE_DATA,       &afe_regs.afe_rsense_data));
    EL_RETURN_IF_NOT_OK(readRegister(AD_AFE_REG_ADDR_NO_PWR_DEFAULT_STATUS,  AD_AFE_REG_SIZE_NO_PWR_DEFAULT_STATUS, &afe_regs.no_pwr_default_status));

    // ADC registers
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_STATUS,        AD_ADC_REG_SIZE_ADC_STATUS,         &adc_regs.adc_status));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_MODE,          AD_ADC_REG_SIZE_ADC_MODE,           &adc_regs.adc_mode));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_INTERFACE,     AD_ADC_REG_SIZE_ADC_INTERFACE,      &adc_regs.adc_interface));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_CONFIG,        AD_ADC_REG_SIZE_ADC_CONFIG,         &adc_regs.adc_config));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_DATA,              AD_ADC_REG_SIZE_DATA,               &adc_regs.data));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_FILTER,            AD_ADC_REG_SIZE_FILTER,             &adc_regs.filter));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_GPIO_CONFIG,   AD_ADC_REG_SIZE_ADC_GPIO_CONFIG,    &adc_regs.adc_gpio_config));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ID,                AD_ADC_REG_SIZE_ID,                 &adc_regs.id));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_OFFSET0,       AD_ADC_REG_SIZE_ADC_OFFSET0,        &adc_regs.adc_offset0));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_OFFSET1,       AD_ADC_REG_SIZE_ADC_OFFSET1,        &adc_regs.adc_offset1));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_OFFSET2,       AD_ADC_REG_SIZE_ADC_OFFSET2,        &adc_regs.adc_offset2));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_OFFSET3,       AD_ADC_REG_SIZE_ADC_OFFSET3,        &adc_regs.adc_offset3));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_GAIN0,         AD_ADC_REG_SIZE_ADC_GAIN0,          &adc_regs.adc_gain0));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_GAIN1,         AD_ADC_REG_SIZE_ADC_GAIN1,          &adc_regs.adc_gain1));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_GAIN2,         AD_ADC_REG_SIZE_ADC_GAIN2,          &adc_regs.adc_gain2));
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_GAIN3,         AD_ADC_REG_SIZE_ADC_GAIN3,          &adc_regs.adc_gain3));

    return el::retcode::ok;
}

el::retcode AD4110::writeAllRegisters()
{
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1,               AD_AFE_REG_SIZE_AFE_CNTRL1,         afe_regs.afe_cntrl1));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CLK_CTRL,             AD_AFE_REG_SIZE_AFE_CLK_CTRL,       afe_regs.afe_clk_ctrl));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2,               AD_AFE_REG_SIZE_AFE_CNTRL2,         afe_regs.afe_cntrl2));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL,             AD_AFE_REG_SIZE_PGA_RTD_CTRL,       afe_regs.pga_rtd_ctrl));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_ERR_DISABLE,          AD_AFE_REG_SIZE_AFE_ERR_DISABLE,    afe_regs.afe_err_disable));
    
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_MODE,         AD_ADC_REG_SIZE_ADC_MODE,           adc_regs.adc_mode));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_INTERFACE,    AD_ADC_REG_SIZE_ADC_INTERFACE,      adc_regs.adc_interface));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG,       AD_ADC_REG_SIZE_ADC_CONFIG,         adc_regs.adc_config));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_FILTER,           AD_ADC_REG_SIZE_FILTER,             adc_regs.filter));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GPIO_CONFIG,  AD_ADC_REG_SIZE_ADC_GPIO_CONFIG,    adc_regs.adc_gpio_config));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_OFFSET0,      AD_ADC_REG_SIZE_ADC_OFFSET0,        adc_regs.adc_offset0));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_OFFSET1,      AD_ADC_REG_SIZE_ADC_OFFSET1,        adc_regs.adc_offset1));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_OFFSET2,      AD_ADC_REG_SIZE_ADC_OFFSET2,        adc_regs.adc_offset2));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_OFFSET3,      AD_ADC_REG_SIZE_ADC_OFFSET3,        adc_regs.adc_offset3));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GAIN0,        AD_ADC_REG_SIZE_ADC_GAIN0,          adc_regs.adc_gain0));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GAIN1,        AD_ADC_REG_SIZE_ADC_GAIN1,          adc_regs.adc_gain1));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GAIN2,        AD_ADC_REG_SIZE_ADC_GAIN2,          adc_regs.adc_gain2));
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GAIN3,        AD_ADC_REG_SIZE_ADC_GAIN3,          adc_regs.adc_gain3));

    return el::retcode::ok;
}

el::retcode AD4110::setup()
{
    // configure ADC to route it's internal clock to CLKIO pin
    AD_WRITE_BITS(adc_regs.adc_mode, AD_MASK_ADC_CLK_SEL, AD_BITS_ADC_CLK_INT_IO);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_MODE, AD_ADC_REG_SIZE_ADC_MODE, adc_regs.adc_mode));

    // configure the AFE to use the clock from the CLKIO pin which is now the ADC clock
    AD_WRITE_BITS(afe_regs.afe_clk_ctrl, AD_MASK_AFE_CLK_CFG, AD_BITS_AFE_CLK_EXT);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CLK_CTRL, AD_AFE_REG_SIZE_AFE_CLK_CTRL, afe_regs.afe_clk_ctrl));

    // enable the error output
    AD_WRITE_BITS(adc_regs.adc_gpio_config, AD_MASK_ERR_PIN_EN, AD_BITS_ERR_PIN_OUT);
    // disable the sync pin so the chip runs without pulling it high
    AD_CLEAR_BITS(adc_regs.adc_gpio_config, AD_MASK_SYNC_PIN_EN);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_GPIO_CONFIG, AD_ADC_REG_SIZE_ADC_GPIO_CONFIG, adc_regs.adc_gpio_config));
    
    // enable appending status register to data register for combined reading
    AD_SET_BITS(adc_regs.adc_interface, AD_MASK_DATA_STAT);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_INTERFACE, AD_ADC_REG_SIZE_ADC_INTERFACE, adc_regs.adc_interface));

    //configure the filter
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_ODR, AD_BITS_ODR_60);
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_FILT_ORDER, AD_BITS_ORD_SINC51);
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_EFILT_SEL, AD_BITS_EFILT_UNDEF);
    AD_CLEAR_BITS(adc_regs.filter, AD_MASK_EFILT_EN);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_FILTER, AD_ADC_REG_SIZE_FILTER, adc_regs.filter));

    // enable bias voltage
    AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_ON50);
    // enable voltage mode regardless of startup value
    AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));

    // enable channel 0
    AD_SET_BITS(adc_regs.adc_config, AD_MASK_CHAN_EN_0);
    AD_CLEAR_BITS(adc_regs.adc_config, AD_MASK_BIT_6);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG, AD_ADC_REG_SIZE_ADC_CONFIG, adc_regs.adc_config));

   

    return el::retcode::ok;
}

el::retcode AD4110::updateStatus()
{
    if (adc_data_transaction_in_progress)
        return el::retcode::busy;
    
    EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_ADC_STATUS,        AD_ADC_REG_SIZE_ADC_STATUS,         &adc_regs.adc_status));
    return el::retcode::ok;
}

static int counter = 0;

el::retcode AD4110::readData(uint32_t *_output)
{
    if (adc_data_transaction_in_progress)
        return el::retcode::busy;
    adc_data_transaction_in_progress = true;
    counter = 0;

    // enable chip and wait for ready to become low
    cs_pin.write(0);
    ready_pin.enableInterrupt();
    auto bits = xEventGroupWaitBits(
        event_group,
        AD_EVG_IT_READY,
        pdTRUE,
        pdFALSE,
        timeout_ticks
    );

    if (!bits)
    {
        adc_data_transaction_in_progress = false;
        return el::retcode::timeout;
    }

    // ready interrupt received, start reading data

    // if DATA_STAT bit is set, the ADC_STATUS register is appended to the data register,
    // making it one byte longer and requiring the two to be split up
    if (AD_TEST_BITS(adc_regs.adc_interface, AD_MASK_DATA_STAT))
    {
        EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_DATA, (ad_reg_size_t)(AD_ADC_REG_SIZE_DATA + AD_ADC_REG_SIZE_ADC_STATUS), &adc_regs.data));
        adc_regs.adc_status = adc_regs.data & 0xff; // lowest byte contains status reg
        adc_regs.data >>= 8;                        // throw out the status reg from the data
    }
    // Otherwise, the data register can just be read like normal
    else
    {
        EL_RETURN_IF_NOT_OK(readRegister(AD_ADC_REG_ADDR_DATA, AD_ADC_REG_SIZE_DATA, &adc_regs.data));
    }

    *_output = adc_regs.data;
    printf("Counter: %d", counter);

    adc_data_transaction_in_progress = false;
    return el::retcode::ok;
}

void AD4110::readyInterruptHandler()
{
    BaseType_t higher_priority_task_woken, result;

    // disable any further interrupts
    ready_pin.disableInterrupt();
    counter++;

    // notify blocking task
    result = xEventGroupSetBitsFromISR(
        event_group,
        AD_EVG_IT_READY,
        &higher_priority_task_woken
    );

    if (result != pdFAIL)
        portYIELD_FROM_ISR(higher_priority_task_woken);
}