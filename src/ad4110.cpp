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

#include "ad4110.hpp"

AD4110::AD4110(
    uint8_t _addr,
    uc::Pin _cs_pin
) :
    address(_addr),
    cs_pin(_cs_pin)
{
    
}