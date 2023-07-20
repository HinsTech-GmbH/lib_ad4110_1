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