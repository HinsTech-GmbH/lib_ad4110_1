/**
 * @file ad4110_init.cpp
 * @author melektron (matteo@elektron.work)
 * @brief initialization functions of AD4110-1 driver class
 * @version 0.1
 * @date 2023-07-27
 * 
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 * 
 */

#include <memory.h>

#include "ad4110.hpp"

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


void AD4110::initialize()
{
    sem_spi_guard = xSemaphoreCreateBinaryStatic(&sem_buffer_spi_guard);
    configASSERT(sem_spi_guard);
    xSemaphoreGive(sem_spi_guard);

    sem_device_guard = xSemaphoreCreateBinaryStatic(&sem_buffer_device_guard);
    configASSERT(sem_device_guard);
    xSemaphoreGive(sem_device_guard);

    event_group = xEventGroupCreateStatic(&event_group_buffer);
    configASSERT(event_group);
}

el::retcode AD4110::reset()
{
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);

    uint8_t data_len = 8;
    memset(spi_write_buffer, 0xff, data_len);

    return transmitBytes(data_len);
}

el::retcode AD4110::loadAllRegisters()
{
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);
    
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
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);
    
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
    ADScopedAccess lock(this);
    EL_RETURN_IF_NOT_OK(lock.status);
    
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
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_ODR, AD_BITS_ODR_1K);
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_FILT_ORDER, AD_BITS_ORD_SINC51);
    AD_WRITE_BITS(adc_regs.filter, AD_MASK_EFILT_SEL, AD_BITS_EFILT_UNDEF);
    AD_CLEAR_BITS(adc_regs.filter, AD_MASK_EFILT_EN);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_FILTER, AD_ADC_REG_SIZE_FILTER, adc_regs.filter));

    // enable voltage mode regardless of startup value
    AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));

    // enable channel 0
    AD_SET_BITS(adc_regs.adc_config, AD_MASK_CHAN_EN_0);
    AD_CLEAR_BITS(adc_regs.adc_config, AD_MASK_BIT_6);
    EL_RETURN_IF_NOT_OK(writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG, AD_ADC_REG_SIZE_ADC_CONFIG, adc_regs.adc_config));


    return el::retcode::ok;
}
