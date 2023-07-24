/**
 * @file ad4110_regs.hpp
 * @author melektron (matteo@elektron.work)
 * @brief register definitions of the AD4110-1 IC
 * @version 0.1
 * @date 2023-07-24
 * 
 * @copyright Copyright HinsTech GmbH (c) 2023-now
 * 
 */

#pragma once

#include <stdint.h>

/**
 * Register size enumeration.
 * Used for reading and writing.
 */

enum ad_reg_size_t
{
    AD_REG_8 = 1,
    AD_REG_16 = 2,
    AD_REG_24 = 3,
};

/**
 * Register definitions.
 * 
 * The ADDR definitions contain the address of the register (low nibble)
 * and the register bank bit (MSB).
 * The register bank bit specifies the following: 
 * 0 -> ADC bank
 * 1 -> AFE bank
 * 
 * The SIZE definition contains the enumeration of the register size
 * (8/16/24 bits)
 */

// AFE registers
#define AD_AFE_REG_ADDR_AFE_TOP_STATUS       0x80
#define AD_AFE_REG_ADDR_AFE_CNTRL1           0x81
#define AD_AFE_REG_ADDR_AFE_CLK_CTRL         0x83
#define AD_AFE_REG_ADDR_AFE_CNTRL2           0x84
#define AD_AFE_REG_ADDR_PGA_RTD_CTRL         0x85
#define AD_AFE_REG_ADDR_AFE_ERR_DISABLE      0x86
#define AD_AFE_REG_ADDR_AFE_DETAIL_STATUS    0x87
#define AD_AFE_REG_ADDR_AFE_CAL_DATA         0x8c
#define AD_AFE_REG_ADDR_AFE_RSENSE_DATA      0x8d
#define AD_AFE_REG_ADDR_NO_PWR_DEFAULT_SEL   0x8e
#define AD_AFE_REG_ADDR_NO_PWR_DEFAULT_STATUS    0x8f
// ADC register sizes
#define AD_AFE_REG_SIZE_AFE_TOP_STATUS       AD_REG_16
#define AD_AFE_REG_SIZE_AFE_CNTRL1           AD_REG_16
#define AD_AFE_REG_SIZE_AFE_CLK_CTRL         AD_REG_16
#define AD_AFE_REG_SIZE_AFE_CNTRL2           AD_REG_16
#define AD_AFE_REG_SIZE_PGA_RTD_CTRL         AD_REG_16
#define AD_AFE_REG_SIZE_AFE_ERR_DISABLE      AD_REG_16
#define AD_AFE_REG_SIZE_AFE_DETAIL_STATUS    AD_REG_16
#define AD_AFE_REG_SIZE_AFE_CAL_DATA         AD_REG_16
#define AD_AFE_REG_SIZE_AFE_RSENSE_DATA      AD_REG_16
#define AD_AFE_REG_SIZE_NO_PWR_DEFAULT_SEL   AD_REG_16
#define AD_AFE_REG_SIZE_NO_PWR_DEFAULT_STATUS    AD_REG_16


// ADC registers
#define AD_ADC_REG_ADDR_ADC_STATUS           0x00
#define AD_ADC_REG_ADDR_ADC_MODE             0x01
#define AD_ADC_REG_ADDR_ADC_INTERFACE        0x02
#define AD_ADC_REG_ADDR_ADC_CONFIG           0x03
#define AD_ADC_REG_ADDR_DATA                 0x04
#define AD_ADC_REG_ADDR_FILTER               0x05
#define AD_ADC_REG_ADDR_ADC_GPIO_CONFIG      0x06
#define AD_ADC_REG_ADDR_ID                   0x07
#define AD_ADC_REG_ADDR_ADC_OFFSET0          0x08
#define AD_ADC_REG_ADDR_ADC_OFFSET1          0x09
#define AD_ADC_REG_ADDR_ADC_OFFSET2          0x0a
#define AD_ADC_REG_ADDR_ADC_OFFSET3          0x0b
#define AD_ADC_REG_ADDR_ADC_GAIN0            0x0c
#define AD_ADC_REG_ADDR_ADC_GAIN1            0x0d
#define AD_ADC_REG_ADDR_ADC_GAIN2            0x0e
#define AD_ADC_REG_ADDR_ADC_GAIN3            0x0f
// ADC register sizes
#define AD_ADC_REG_SIZE_ADC_STATUS           AD_REG_8
#define AD_ADC_REG_SIZE_ADC_MODE             AD_REG_16
#define AD_ADC_REG_SIZE_ADC_INTERFACE        AD_REG_16
#define AD_ADC_REG_SIZE_ADC_CONFIG           AD_REG_16
#define AD_ADC_REG_SIZE_DATA                 AD_REG_24
#define AD_ADC_REG_SIZE_FILTER               AD_REG_16
#define AD_ADC_REG_SIZE_ADC_GPIO_CONFIG      AD_REG_16
#define AD_ADC_REG_SIZE_ID                   AD_REG_16
#define AD_ADC_REG_SIZE_ADC_OFFSET0          AD_REG_24
#define AD_ADC_REG_SIZE_ADC_OFFSET1          AD_REG_24
#define AD_ADC_REG_SIZE_ADC_OFFSET2          AD_REG_24
#define AD_ADC_REG_SIZE_ADC_OFFSET3          AD_REG_24
#define AD_ADC_REG_SIZE_ADC_GAIN0            AD_REG_24
#define AD_ADC_REG_SIZE_ADC_GAIN1            AD_REG_24
#define AD_ADC_REG_SIZE_ADC_GAIN2            AD_REG_24
#define AD_ADC_REG_SIZE_ADC_GAIN3            AD_REG_24

/**
 * Register cache types.
 * 
 * These types are used to store a cache of all the register values
 * locally so the register doesn't have to be read just to modify a single bit.
 */

// cache type for all AFE regs
struct ad_regs_afe_t
{
    uint32_t afe_top_status;
    uint32_t afe_cntrl1;
    uint32_t afe_clk_ctrl;
    uint32_t afe_cntrl2;
    uint32_t pga_rtd_ctrl;
    uint32_t afe_err_disable;
    uint32_t afe_detail_status;
    uint32_t afe_cal_data;
    uint32_t afe_rsense_data;
    uint32_t no_pwr_default_status;
};

// cache type for all ADC regs
struct ad_regs_adc_t
{
    uint32_t adc_status;
    uint32_t adc_mode;
    uint32_t adc_interface;
    uint32_t adc_config;
    uint32_t data;
    uint32_t filter;
    uint32_t adc_gpio_config;
    uint32_t id;
    uint32_t adc_offset0;
    uint32_t adc_offset1;
    uint32_t adc_offset2;
    uint32_t adc_offset3;
    uint32_t adc_gain0;
    uint32_t adc_gain1;
    uint32_t adc_gain2;
    uint32_t adc_gain3;
};

/**
 * Register value definitions.
 * 
 * These definitions define special values for some registers
 * or masks specifying the relevant bits for a function.
 * 
 */

// AFE Registers

// AFE_TOP_STATUS Register
#define AD_MASK_ERRCH (1 << 5)      // AFE_TOP_STATUS Error on channel (non-maskable)
#define AD_MASK_ERRCRC (1 << 4)     // AFE_TOP_STATUS CRC check failed
#define AD_MASK_TEMPSD (1 << 3)     // AFE_TOP_STATUS Thermal shutdown
#define AD_MASK_TEMPHI (1 << 2)     // AFE_TOP_STATUS Overtemperature detection 
#define AD_MASK_AFE_ERROR (1)       // AFE_TOP_STATUS Error on channel (masked, state of ERR pin)

// AFE_CNTRL1 Register
#define AD_MASK_CRC_EN (1 << 14)    // AFE_CNTRL1 CRC Enable
#define AD_MASK_DISRTD (1 << 9)     // AFE_CNTRL1 Disable RTD currents

// AFE_CLK_CTRL Register
#define AD_MASK_CLK_CFG (1 << 4)    // AFE_CLK_CTRL Clock pin configuration (basically AFE clock enable since it must be written to 1 to work properly according to datasheet)

// AFE_CNTRL2 Register
#define AD_MASK_AINN_DN100 (1 << 15)    // AFE_CNTRL2 N Input open wire detection current enable (-100µA)
#define AD_MASK_AINN_DN1   (1 << 14)    // AFE_CNTRL2 N Input open wire detection current enable (-1µA)
#define AD_MASK_AINN_UP100 (1 << 13)    // AFE_CNTRL2 N Input open wire detection current enable (100µA)
#define AD_MASK_AINN_UP1   (1 << 12)    // AFE_CNTRL2 N Input open wire detection current enable (1µA)
#define AD_MASK_AINP_DN100 (1 << 11)    // AFE_CNTRL2 P Input open wire detection current enable (-100µA)
#define AD_MASK_AINP_DN1   (1 << 10)    // AFE_CNTRL2 P Input open wire detection current enable (-1µA)
#define AD_MASK_AINP_UP100 (1 << 9)     // AFE_CNTRL2 P Input open wire detection current enable (100µA)
#define AD_MASK_AINP_UP1   (1 << 8)     // AFE_CNTRL2 P Input open wire detection current enable (1µA)
#define AD_MASK_VBIAS      (11 << 6)    // AFE_CNTRL2 Common-mode bias voltage config
#define AD_MASK_EN_FLD_PWR (1 << 3)     // AFE_CNTRL2 Enable field power supply mode
#define AD_MASK_EXT_R_SEL  (1 << 2)     // AFE_CNTRL2 External current sense resistor select (1 = external, 0 = internal)
#define AD_MASK_IMODE      (1 << 1)     // AFE_CNTRL2 Current mode enabled (0 = Voltage mode, 1 = Current mode)

// TODO: continue here with AFE_CNTRL2 vbias values