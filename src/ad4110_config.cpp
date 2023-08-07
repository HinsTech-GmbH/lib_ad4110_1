/**
 * @file ad4110_config.cpp
 * @author melektron (matteo@elektron.work)
 * @brief user configuration functions
 * @version 0.1
 * @date 2023-07-27
 *
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 *
 */

#include "ad4110.hpp"

el::retcode AD4110::setInputMode(input_mode_t _input_mode)
{
    switch (_input_mode)
    {

    case input_mode_t::IM_VOLTAGE:
        AD_CLEAR_BITS(                                                                  // turn off any pull ups or pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN100 |
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_0p2);        // turn the gain down BEFORE enabling voltage mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_OFF);           // turn off the bias voltage
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);              // select voltage mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_SET_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                               // disable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        break;

    case input_mode_t::IM_CURRENT:
        AD_CLEAR_BITS(                                                                  // turn off any pull ups or pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN100 |
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_OFF);           // turn off the bias voltage
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_I);              // select current mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_SET_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                               // disable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_4);          // turn the gain up AFTER enabling current mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        break;

    case input_mode_t::IM_RTD2W:
        AD_SET_BITS(afe_regs.afe_cntrl2, AD_MASK_AINN_DN100);                           // turn on the 100µA pull down on the AIN(-) pin
        AD_CLEAR_BITS(                                                                  // turn off all other pull ups and pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_0p2);        // turn the gain down BEFORE enabling voltage mode for safety
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_ON50);          // turn bias voltage on
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);              // select voltage mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_CLEAR_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                             // enable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_RTD_3W4W, AD_BITS_RTD_23W);        // 2/3 wire mode  
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_EXC_SEL, AD_BITS_RTD_EXC_100);   // excitation current 100µA
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_COM_SEL, AD_BITS_RTD_COM_OFF);   // compensation current off
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        break;

    case input_mode_t::IM_RTD3W:
        AD_CLEAR_BITS(                                                                  // turn off any pull ups or pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN100 |
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_0p2);        // turn the gain down BEFORE enabling voltage mode for safety
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_OFF);           // turn bias voltage off
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);              // select voltage mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_CLEAR_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                             // enable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_RTD_3W4W, AD_BITS_RTD_23W);        // 2/3 wire mode  
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_EXC_SEL, AD_BITS_RTD_EXC_600);   // excitation current 600µA
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_COM_SEL, AD_BITS_RTD_COM_600);   // compensation current also 600µA
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        break;

    case input_mode_t::IM_RTD4W:
        AD_CLEAR_BITS(                                                                  // turn off any pull ups or pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN100 |
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_0p2);        // turn the gain down BEFORE enabling voltage mode for safety
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_OFF);           // turn bias voltage off
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);              // select voltage mode
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_CLEAR_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                             // enable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_RTD_3W4W, AD_BITS_RTD_4W);         // 4 wire mode  
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_EXC_SEL, AD_BITS_RTD_EXC_600);   // excitation current 600µA
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_I_COM_SEL, AD_BITS_RTD_COM_OFF);   // compensation current off (goes to GND in 4 wire mode)
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));

        break;

    case input_mode_t::IM_THERMO:
        AD_CLEAR_BITS(                                                                  // turn off any pull ups or pull downs
            afe_regs.afe_cntrl2,
            AD_MASK_AINN_DN100 |
            AD_MASK_AINN_DN1 |
            AD_MASK_AINN_UP100 |
            AD_MASK_AINN_UP1 |
            AD_MASK_AINP_DN100 |
            AD_MASK_AINP_DN1 |
            AD_MASK_AINP_UP100 |
            AD_MASK_AINP_UP1
        );
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, AD_BITS_GAIN_0p2);        // turn the gain down BEFORE enabling voltage mode for safety
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl));
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, AD_BITS_VBIAS_ON50);
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_IMODE, AD_BITS_MODE_V);
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2));
        AD_SET_BITS(afe_regs.afe_cntrl1, AD_MASK_DISRTD);                               // disable RTD feature
        EL_RETURN_IF_NOT_OK(writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL1, AD_AFE_REG_SIZE_AFE_CNTRL1, afe_regs.afe_cntrl1));
        break;

    default:
        return el::retcode::invalid;
        break;
    }

    return el::retcode::ok;
}

el::retcode AD4110::selectRSense(int_ext_t _resistor)
{
    if (_resistor == INTERNAL)
    {
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_EXT_R_SEL, AD_BITS_RSENSE_INT);
    }
    else if (_resistor == EXTERNAL)
    {
        AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_EXT_R_SEL, AD_BITS_RSENSE_EXT);
    }
    else
    {
        return el::retcode::invalid;
    }

    return writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2);
}

el::retcode AD4110::selectRRTD(int_ext_t _resistor)
{
    if (_resistor == INTERNAL)
    {
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_EXT_RTD_RES, AD_BITS_RTD_RES_INT);
    }
    else if (_resistor == EXTERNAL)
    {
        AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_EXT_RTD_RES, AD_BITS_RTD_RES_EXT);
    }
    else
    {
        return el::retcode::invalid;
    }

    return writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl);
}

el::retcode AD4110::selectVoltageReference(int_ext_t _reference)
{
    if (_reference == INTERNAL)
    {
        AD_WRITE_BITS(adc_regs.adc_config, AD_MASK_REF_SEL, AD_BITS_REF_INT);
    }
    else if (_reference == EXTERNAL)
    {
        AD_WRITE_BITS(adc_regs.adc_config, AD_MASK_REF_SEL, AD_BITS_REF_EXT);
    }
    // there is also the AVDD5 reference option but we don't support it atm
    else
    {
        return el::retcode::invalid;
    }

    return writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG, AD_ADC_REG_SIZE_ADC_CONFIG, adc_regs.adc_config);
}

static uint8_t channel_bits[] = {
    AD_MASK_CHAN_EN_0,
    AD_MASK_CHAN_EN_1,
    AD_MASK_CHAN_EN_2,
    AD_MASK_CHAN_EN_3
};
el::retcode AD4110::enableChannel(channel_t _ch)
{
    if (_ch < 0 || _ch > 3) return el::retcode::invalid;

    AD_SET_BITS(adc_regs.adc_config, channel_bits[_ch]);

    return writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG, AD_ADC_REG_SIZE_ADC_CONFIG, adc_regs.adc_config);
}
el::retcode AD4110::disableChannel(channel_t _ch)
{
    if (_ch < 0 || _ch > 3) return el::retcode::invalid;

    AD_CLEAR_BITS(adc_regs.adc_config, channel_bits[_ch]);

    return writeRegister(AD_ADC_REG_ADDR_ADC_CONFIG, AD_ADC_REG_SIZE_ADC_CONFIG, adc_regs.adc_config);
}

static uint8_t sample_rate_bits[] = {
    AD_BITS_ODR_125K,
    AD_BITS_ODR_62K5,
    AD_BITS_ODR_31K2,
    AD_BITS_ODR_25K,
    AD_BITS_ODR_15K6,
    AD_BITS_ODR_10K4,
    AD_BITS_ODR_5K,
    AD_BITS_ODR_2K5,
    AD_BITS_ODR_1K,
    AD_BITS_ODR_500,
    AD_BITS_ODR_400,
    AD_BITS_ODR_200,
    AD_BITS_ODR_100,
    AD_BITS_ODR_60,
    AD_BITS_ODR_50,
    AD_BITS_ODR_20,
    AD_BITS_ODR_16,
    AD_BITS_ODR_10,
    AD_BITS_ODR_5
};
el::retcode AD4110::setSampleRate(sample_rate_t _sr)
{
    if (_sr < 0 || _sr > SR_5_SPS) return el::retcode::invalid;

    AD_WRITE_BITS(adc_regs.filter, AD_MASK_ODR, sample_rate_bits[_sr]);

    return writeRegister(AD_ADC_REG_ADDR_FILTER, AD_ADC_REG_SIZE_FILTER, adc_regs.filter);
}

el::retcode AD4110::setBiasVoltage(bool _on)
{
    AD_WRITE_BITS(afe_regs.afe_cntrl2, AD_MASK_VBIAS, _on ? AD_BITS_VBIAS_ON50 : AD_BITS_VBIAS_OFF);
    
    return writeRegister(AD_AFE_REG_ADDR_AFE_CNTRL2, AD_AFE_REG_SIZE_AFE_CNTRL2, afe_regs.afe_cntrl2);
}

el::retcode AD4110::setGain(gain_t _gain)
{
    AD_WRITE_BITS(afe_regs.pga_rtd_ctrl, AD_MASK_GAIN_CH, (int)_gain);  // the gain_t enumerates to the correct bit values atm
    
    return writeRegister(AD_AFE_REG_ADDR_PGA_RTD_CTRL, AD_AFE_REG_SIZE_PGA_RTD_CTRL, afe_regs.pga_rtd_ctrl);
}