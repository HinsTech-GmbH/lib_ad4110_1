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

#pragma once

#include <stdint.h>
#include <uc_gpio.hpp>


#define AD_READ         0b01000000
#define AD_WRITE        0b00000000
#define AD_ADDR(addr)   ((addr) << 4)

// Registers: MSB specifies AFE(1)/ADC(0), 4 LSBs register address
// AFE registers
#define AD_AFE_REG_AFE_TOP_STATUS       0x80
#define AD_AFE_REG_AFE_CNTRL1           0x81
// reserved 0x82
#define AD_AFE_REG_AFE_CLK_CTRL         0x83
#define AD_AFE_REG_AFE_CNTRL2           0x84
#define AD_AFE_REG_PGA_RTD_CTRL         0x85
#define AD_AFE_REG_AFE_ERR_DISABLE      0x86
#define AD_AFE_REG_AFE_DETAIL_STATUS    0x87
// reserved 0x88
// reserved 0x89
// reserved 0x8a
// reserved 0x8b
#define AD_AFE_REG_AFE_CAL_DATA         0x8c
#define AD_AFE_REG_AFE_RSENSE_DATA      0x8d
#define AD_AFE_REG_NO_PWR_DEFAULT_SEL   0x8e
#define AD_AFE_REG_NO_PWR_DEFAULT_STATUS    0x8f

// ADC registers
#define AD_ADC_REG_ADC_STATUS           0x00
#define AD_ADC_REG_ADC_MODE             0x01
#define AD_ADC_REG_ADC_INTERFACE        0x02
#define AD_ADC_REG_ADC_CONFIG           0x03
#define AD_ADC_REG_DATA                 0x04
#define AD_ADC_REG_FILTER               0x05
#define AD_ADC_REG_ADC_GPIO_CONFIG      0x06
#define AD_ADC_REG_ID                   0x07
#define AD_ADC_REG_ADC_OFFSET0          0x08
#define AD_ADC_REG_ADC_OFFSET1          0x09
#define AD_ADC_REG_ADC_OFFSET2          0x0a
#define AD_ADC_REG_ADC_OFFSET3          0x0b
#define AD_ADC_REG_ADC_GAIN0            0x0c
#define AD_ADC_REG_ADC_GAIN1            0x0d
#define AD_ADC_REG_ADC_GAIN2            0x0e
#define AD_ADC_REG_ADC_GAIN3            0x0f


class AD4110
{
private:
    // The address of the AD4110-1 chip on the bus (this chip has a special way of using addresses over SPI)
    uint8_t address;
    // The CS pin the AD4110-1 is connected to (can be shared with multiple AD4110-1 if they have different pins)
    uc::Pin cs_pin;

public:

    /**
     * @brief Construct a new AD4110 driver instance
     *
     * @param _addr The address of the AD4110-1 chip on the bus (this chip has a special way of using addresses over SPI)
     * @param _cs_pin The CS pin the AD4110-1 is connected to (can be shared with multiple AD4110-1 if they have different pins)
     */
    AD4110(
        uint8_t _addr,
        uc::Pin _cs_pin
    );
};