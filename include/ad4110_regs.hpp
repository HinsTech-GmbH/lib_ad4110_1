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
#include <stdbool.h>

/**
 * Bit utilities.
 * 
 * Macros to simplify bitwise operations on the 
 * registers.
 */
// evaluates to true if any of the bits defined in mask are set in data
#define AD_TEST_BITS(data, mask) (bool)((data) & (mask))
// sets the bits defined by "mask" in "data" (aka. sets them to 1)
#define AD_SET_BITS(data, mask) (data) |= (mask)
// clears the bits defined by "mask" in "data" (aka. sets them to 0)
#define AD_CLEAR_BITS(data, mask) (data) &= ~(mask)
// sets the bits in "data" defined by "mask" to bits in "value" at that mask: AD_WRITE_BITS(0b010101, 0b001100, 0b001000) -> 0b011001 (bits 3:2 were copied to "data" from "value")
#define AD_WRITE_BITS(data, mask, value) (data) = ((data) & ~(mask)) | ((value) & (mask))


/**
 * Register size enumeration.
 * Used for reading and writing.
 */

enum ad_reg_size_t
{
    AD_REG_8 = 1,
    AD_REG_16 = 2,
    AD_REG_24 = 3,
    AD_REG_32 = 4
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
#define AD_AFE_REG_ADDR_AFE_CAL_DATA         0x8C
#define AD_AFE_REG_ADDR_AFE_RSENSE_DATA      0x8D
#define AD_AFE_REG_ADDR_NO_PWR_DEFAULT_SEL   0x8E
#define AD_AFE_REG_ADDR_NO_PWR_DEFAULT_STATUS    0x8F
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
#define AD_ADC_REG_ADDR_ADC_OFFSET2          0x0A
#define AD_ADC_REG_ADDR_ADC_OFFSET3          0x0B
#define AD_ADC_REG_ADDR_ADC_GAIN0            0x0C
#define AD_ADC_REG_ADDR_ADC_GAIN1            0x0D
#define AD_ADC_REG_ADDR_ADC_GAIN2            0x0E
#define AD_ADC_REG_ADDR_ADC_GAIN3            0x0F
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
#define AD_MASK_ERRCH       (0b1 << 5)      // AFE_TOP_STATUS Error on channel (non-maskable)
#define AD_MASK_AFE_ERRCRC  (0b1 << 4)      // AFE_TOP_STATUS CRC check failed for AFE register write
#define AD_MASK_TEMPSD      (0b1 << 3)      // AFE_TOP_STATUS Thermal shutdown
#define AD_MASK_TEMPHI      (0b1 << 2)      // AFE_TOP_STATUS Overtemperature detection 
#define AD_MASK_AFE_ERROR   (0b1)           // AFE_TOP_STATUS Error on channel (masked, state of ERR pin)

// AFE_CNTRL1 Register
#define AD_MASK_AFE_CRC_EN  (0b1 << 14)     // AFE_CNTRL1 CRC enable for AFE register read/write
#define AD_MASK_DISRTD      (0b1 << 9)      // AFE_CNTRL1 Disable RTD currents

// AFE_CLK_CTRL Register
#define AD_MASK_AFE_CLK_CFG (0b1 << 4)      // AFE_CLK_CTRL Clock pin configuration (basically AFE clock enable since it must be written to 1 to work properly according to datasheet)

#define AD_BITS_AFE_CLK_INT (0b0 << 4)      // AFE_CLK_CTRL AFE clocked internally (default)
#define AD_BITS_AFE_CLK_EXT (0b1 << 4)      // AFE_CLK_CTRL AFE clocked by CLKIO pin (which can in turn be clocked by ADC)

// AFE_CNTRL2 Register
#define AD_MASK_AINN_DN100  (0b1 << 15)     // AFE_CNTRL2 N Input open wire detection current enable (-100µA)
#define AD_MASK_AINN_DN1    (0b1 << 14)     // AFE_CNTRL2 N Input open wire detection current enable (-1µA)
#define AD_MASK_AINN_UP100  (0b1 << 13)     // AFE_CNTRL2 N Input open wire detection current enable (100µA)
#define AD_MASK_AINN_UP1    (0b1 << 12)     // AFE_CNTRL2 N Input open wire detection current enable (1µA)
#define AD_MASK_AINP_DN100  (0b1 << 11)     // AFE_CNTRL2 P Input open wire detection current enable (-100µA)
#define AD_MASK_AINP_DN1    (0b1 << 10)     // AFE_CNTRL2 P Input open wire detection current enable (-1µA)
#define AD_MASK_AINP_UP100  (0b1 << 9)      // AFE_CNTRL2 P Input open wire detection current enable (100µA)
#define AD_MASK_AINP_UP1    (0b1 << 8)      // AFE_CNTRL2 P Input open wire detection current enable (1µA)
#define AD_MASK_VBIAS       (0b11 << 6)     // AFE_CNTRL2 Common-mode bias voltage config
#define AD_MASK_EN_FLD_PWR  (0b1 << 3)      // AFE_CNTRL2 Enable field power supply mode
#define AD_MASK_EXT_R_SEL   (0b1 << 2)      // AFE_CNTRL2 External current sense resistor select (1 = external, 0 = internal)
#define AD_MASK_IMODE       (0b1 << 1)      // AFE_CNTRL2 Current mode enabled (0 = Voltage mode, 1 = Current mode)

#define AD_BITS_VBIAS_ON50  (0b01 << 6)     // AFE_CNTRL2 Vbias On 50 µA 
#define AD_BITS_VBIAS_OFF   (0b10 << 6)     // AFE_CNTRL2 Vbias Off value
#define AD_BITS_RSENSE_EXT  (0b1 << 2)      // AFE_CNTRL2 Sense resistor external
#define AD_BITS_RSENSE_INT  (0b0 << 2)      // AFE_CNTRL2 Sense resistor internal
#define AD_BITS_MODE_I      (0b1 << 1)      // AFE_CNTRL2 Current mode
#define AD_BITS_MODE_V      (0b0 << 1)      // AFE_CNTRL2 Voltage mode

// PGA_RTD_CTRL Register
#define AD_MASK_RTD_3W4W    (0b1 << 15)     // PGA_RTD_CTRL Configure for 2/3 Wire (1) or 4 wire (0) RTD mode
#define AD_MASK_I_COM_SEL   (0b111 << 12)   // PGA_RTD_CTRL RTD Compensation current selection
#define AD_MASK_I_EXC_SEL   (0b111 << 9)    // PGA_RTD_CTRL RTD Excitation current selection
#define AD_MASK_EXT_RTD_RES (0b1 << 8)      // PGA_RTD_CTRL External RTD resistor enabled (1 = external, 0 = internal)
#define AD_MASK_GAIN_CH     (0b1111 << 4)   // PGA_RTD_CTRL Channel gain configuration

#define AD_BITS_RTD_23W     (0b1 << 15)     // PGA_RTD_CTRL RTD 2/3 Wire mode
#define AD_BITS_RTD_4W      (0b0 << 15)     // PGA_RTD_CTRL RTD 4 Wire mode
#define AD_BITS_RTD_COM_OFF     (0b000 << 12)   // PGA_RTD_CTRL Compensation current off
#define AD_BITS_RTD_COM_100     (0b001 << 12)   // PGA_RTD_CTRL Compensation current 100 µA
#define AD_BITS_RTD_COM_400     (0b010 << 12)   // PGA_RTD_CTRL Compensation current 400 µA
#define AD_BITS_RTD_COM_500     (0b011 << 12)   // PGA_RTD_CTRL Compensation current 500 µA
#define AD_BITS_RTD_COM_600     (0b101 << 12)   // PGA_RTD_CTRL Compensation current 600 µA
#define AD_BITS_RTD_COM_900     (0b110 << 12)   // PGA_RTD_CTRL Compensation current 900 µA
#define AD_BITS_RTD_COM_1000    (0b111 << 12)   // PGA_RTD_CTRL Compensation current 1000 µA
#define AD_BITS_RTD_EXC_OFF     (0b000 << 9)    // PGA_RTD_CTRL Excitation current off
#define AD_BITS_RTD_EXC_100     (0b001 << 9)    // PGA_RTD_CTRL Excitation current 100 µA
#define AD_BITS_RTD_EXC_400     (0b010 << 9)    // PGA_RTD_CTRL Excitation current 400 µA
#define AD_BITS_RTD_EXC_500     (0b011 << 9)    // PGA_RTD_CTRL Excitation current 500 µA
#define AD_BITS_RTD_EXC_600     (0b101 << 9)    // PGA_RTD_CTRL Excitation current 600 µA
#define AD_BITS_RTD_EXC_900     (0b110 << 9)    // PGA_RTD_CTRL Excitation current 900 µA
#define AD_BITS_RTD_EXC_1000    (0b111 << 9)    // PGA_RTD_CTRL Excitation current 1000 µA
#define AD_BITS_RTD_RES_EXT     (0b1 << 8)  // PGA_RTD_CTRL External RTD resistor
#define AD_BITS_RTD_RES_INT     (0b0 << 8)  // PGA_RTD_CTRL Internal RTD resistor

#define AD_BITS_GAIN_0p2    (0b0000 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.2
#define AD_BITS_GAIN_0p25   (0b0001 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.25
#define AD_BITS_GAIN_0p3    (0b0010 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.3
#define AD_BITS_GAIN_0p375  (0b0011 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.375
#define AD_BITS_GAIN_0p5    (0b0100 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.5
#define AD_BITS_GAIN_0p75   (0b0101 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 0.75
#define AD_BITS_GAIN_1      (0b0110 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 1
#define AD_BITS_GAIN_1p5    (0b0111 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 1.5
#define AD_BITS_GAIN_2      (0b1000 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 2
#define AD_BITS_GAIN_3      (0b1001 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 3
#define AD_BITS_GAIN_4      (0b1010 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 4
#define AD_BITS_GAIN_6      (0b1011 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 6
#define AD_BITS_GAIN_8      (0b1100 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 8
#define AD_BITS_GAIN_12     (0b1101 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 12
#define AD_BITS_GAIN_16     (0b1110 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 16
#define AD_BITS_GAIN_24     (0b1111 << 4)   // PPGA_RTD_CTRL Gain selection Gain = 24

// AFE_ERR_DISABLE and AFE_DETAIL_STATUS Register
#define AD_MASK_AINN_UV     (0b1 << 11)     // AFE_ERR_DISABLE/AFE_DETAIL_STATUS AIN(-) Undervoltage bit
#define AD_MASK_AINP_UV     (0b1 << 10)     // AFE_ERR_DISABLE/AFE_DETAIL_STATUS AIN(+) Undervoltage bit
#define AD_MASK_AINN_OV     (0b1 << 9)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS AIN(-) Overvoltage bit
#define AD_MASK_AINP_OV     (0b1 << 8)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS AIN(+) Overvoltage bit
#define AD_MASK_I_EXC_ERR   (0b1 << 7)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS RTD excitation current bit
#define AD_MASK_I_COM_ERR   (0b1 << 6)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS RTD compensation current bit
#define AD_MASK_FLD_PWR_OC  (0b1 << 2)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS Field power overcurrent bit
#define AD_MASK_AIN_OC      (0b1 << 1)      // AFE_ERR_DISABLE/AFE_DETAIL_STATUS Input overcurrent
// AFE_DETAIL_STATUS Register
#define AD_MASK_ERROR       (0b1)           // AFE_DETAIL_STATUS Error on hight voltage channel bit
// AFE_DETAIL_STATUS value note: all fags -> 1 = error, 0 = no error

// AFE_CAL_DATA Register
#define AD_MASK_GAIN_PARITY (0b1 << 9)      // AFE_CAL_DATA Parity bit (0 = even, 1 = odd)
#define AD_MASK_GAIN_CAL    (0x1ff)         // AFE_CAL_DATA Gain calibration data for voltage mode

// AFE_RSENSE_DATA Register
#define AD_MASK_RSENSE_PARITY   (0b1 << 15) // AFE_RSENSE_DATA Parity bit (0 = even, 1 = odd)
#define AD_MASK_RSENSE_CAL  (0x7FFF)        // AFE_RSENSE_DATA Sense resistor calibration data for current mode

// NO_PWR_DEFAULT_SEL Register
#define AD_MASK_D_MODE      (0xFF)          // NO_PWR_DEFAULT_SEL Default setting command byte

#define AD_BITS_DSEL_STORE  (0xB1)          // NO_PWR_DEFAULT_SEL Store default command
#define AD_BITS_DSEL_REFRESH    (0xA1)      // NO_PWR_DEFAULT_SEL Refresh default command

// NO_PWR_DEFAULT_STATUS Register
#define AD_MASK_COMM_ERR    (0b1 << 8)      // NO_PWR_DEFAULT_STATUS Command error bit (1 when there was a problem reading the register value)
#define AD_MASK_DSEL_COUNT  (0xFF)          // NO_PWR_DEFAULT_STATUS Count of remaining default write cycles

// ADC_STATUS Register
#define AD_MASK_DATA_READY  (0b1 << 7)      // ADC_STATUS Ready flag (inverted): 0 = result available, 1 = waiting for new data result
#define AD_MASK_ADC_ERR     (0b1 << 6)      // ADC_STATUS ADC Error flag
#define AD_MASK_ADC_CRC_ERR (0b1 << 5)      // ADC_STATUS CRC Error flag while writing ADC register
#define AD_MASK_CHAN_ID     (0b11)          // ADC_STATUS AFE Channel ID

#define AD_CHAN_ID_HV       0b00            // ADC_STATUS 
#define AD_CHAN_ID_HVD      0b00            // ADC_STATUS ID of channel 0 (high voltage differential channel): AIN(+) − AIN(−).
#define AD_CHAN_ID_LVD      0b01            // ADC_STATUS ID of channel 1 (low voltage differential channel): AIN1(LV) − AIN2(LV).
#define AD_CHAN_ID_LV1      0b10            // ADC_STATUS ID of channel 2 (low voltage single-ended channel): AIN1(LV) − AINCOM(LV).
#define AD_CHAN_ID_LV2      0b11            // ADC_STATUS ID of channel 3 (low voltage single-ended channel): AIN2(LV) − AINCOM(LV).

// ADC_MODE Register
#define AD_MASK_REF_EN      (0b1 << 15)     // ADC_MODE Internal voltage reference enable flag
#define AD_MASK_CONV_DELAY  (0b111 << 8)    // ADC_MODE Conversion delay config
#define AD_MASK_CONV_MODE   (0b111 << 4)    // ADC_MODE Conversion mode config
#define AD_MASK_ADC_CLK_SEL (0b11 << 2)     // ADC_MODE ADC Clock source select

#define AD_BITS_DELAY_Off   (0b000 << 8)    // ADC_MODE Conversion delay Off (no delay)
#define AD_BITS_DELAY_1     (0b001 << 8)    // ADC_MODE Conversion delay 1 cycle (8 µs)
#define AD_BITS_DELAY_4     (0b010 << 8)    // ADC_MODE Conversion delay 4 cycles (32 µs)
#define AD_BITS_DELAY_10    (0b011 << 8)    // ADC_MODE Conversion delay 10 cycles (40 µs)
#define AD_BITS_DELAY_25    (0b100 << 8)    // ADC_MODE Conversion delay 25 cycles (200 µs)
#define AD_BITS_DELAY_50    (0b101 << 8)    // ADC_MODE Conversion delay 50 cycles (400 µs)
#define AD_BITS_DELAY_125   (0b110 << 8)    // ADC_MODE Conversion delay 125 cycles (1000 µs)
#define AD_BITS_DELAY_250   (0b111 << 8)    // ADC_MODE Conversion delay 250 cycles (2000 µs)

#define AD_BITS_MODE_CONT       (0b000 << 4)    // ADC_MODE Continuous conversion operation mode
#define AD_BITS_MODE_SINGLE     (0b001 << 4)    // ADC_MODE Single conversion operation mode 
#define AD_BITS_MODE_STANDBY    (0b010 << 4)    // ADC_MODE Standby operation mode
#define AD_BITS_MODE_POWERDOWN  (0b011 << 4)    // ADC_MODE Power-down operation mode
#define AD_BITS_MODE_OFFSET_CAL (0b110 << 4)    // ADC_MODE System offset calibration operation mode
#define AD_BITS_MODE_GAIN_CAL   (0b111 << 4)    // ADC_MODE System gain calibration operation mode

#define AD_BITS_ADC_CLK_INT     (0b00 << 2)     // ADC_MODE ADC Internal clock
#define AD_BITS_ADC_CLK_INT_IO  (0b01 << 2)     // ADC_MODE ADC Internal clock but connected to CLKIO pin (needed for AFE to operate properly)
#define AD_BITS_ADC_CLK_EXT     (0b10 << 2)     // ADC_MODE ADC External clock connected to CLKIO piN

// ADC_INTERFACE Register
#define AD_MASK_DATA_STAT   (0b1 << 6)      // ADC_INTERFACE Append status register to conversion data flag (using this, status register can be read in the same SPI transfer as data to ensure channel information is correct) 
#define AD_MASK_ADC_CRC_CFG (0b11 << 2)     // ADC_INTERFACE CRC configuration for ADC register read/write (unlike AFE CRC config, this has one additional setting)
#define AD_MASK_WL16        (0b1)           // ADC_INTERFACE Flag to use 16 Bit ADC data word length instead of 24 bits (reduces data register size to 16 bits)

#define AD_BITS_ADC_CRC_OFF (0b00 << 2)     // ADC_INTERFACE CRC off configuration (not CRC on ADC register access)
#define AD_BITS_ADC_CRC_XOR (0b01 << 2)     // ADC_INTERFACE XOR checksum on ADC register reads, CRC on ADC register writes
#define AD_BITS_ADC_CRC_ON  (0b10 << 2)     // ADC_INTERFACE CRC on ADC register reads and writes

// ADC_CONFIG Register
#define AD_MASK_BIPOLAR     (0b1 << 12)     // ADC_CONFIG Use bipolar coding
#define AD_MASK_REFIN_BUFF_P    (0b1 << 11)     // ADC_CONFIG Positive reference input buffer enable
#define AD_MASK_REFIN_BUFF_N    (0b1 << 10)     // ADC_CONFIG Negative reference input buffer enable
#define AD_MASK_AIN_BUFF_P  (0b1 << 9)      // ADC_CONFIG Positive main analog input buffer enable
#define AD_MASK_AIN_BUFF_N  (0b1 << 8)      // ADC_CONFIG Negative main analog input buffer enable
#define AD_MASK_BIT_6       (0b1 << 6)      // ADC_CONFIG Bit 6, has to be set to 1 for some reason according to datasheet
#define AD_MASK_REF_SEL     (0b11 << 4)     // ADC_CONFIG Voltage reference selection
#define AD_MASK_CHAN_EN_3   (0b1 << 3)      // ADC_CONFIG Channel 3 enable (low voltage single-ended channel AIN2(LV) − AINCOM(LV))
#define AD_MASK_CHAN_EN_2   (0b1 << 2)      // ADC_CONFIG Channel 2 enable (low voltage single-ended channel: AIN1(LV) − AINCOM(LV))
#define AD_MASK_CHAN_EN_1   (0b1 << 1)      // ADC_CONFIG Channel 1 enable (low voltage differential channel: AIN1(LV) − AIN2(LV))
#define AD_MASK_CHAN_EN_0   (0b1)           // ADC_CONFIG Channel 0 enable (high voltage differential channel: AIN(+) − AIN(−))

#define AD_BITS_REF_EXT     (0b00 << 4)     // ADC_CONFIG External voltage reference connected to REFIN(+) and REFIN(-)
#define AD_BITS_REF_INT     (0b10 << 4)     // ADC_CONFIG Internal voltage reference
#define AD_BITS_REF_AVDD5   (0b11 << 4)     // ADC_CONFIG Use AVDD5 and AGND as voltage reference

// FILTER Register
#define AD_MASK_EFILT_EN    (0b1 << 11)     // FILTER enhanced 50/60 Hz Filter enable
#define AD_MASK_EFILT_SEL   (0b111 << 8)    // FILTER enhanced 50/60 Hz filter mode selection
#define AD_MASK_FILT_ORDER  (0b11 << 5)     // FILTER Order setting
#define AD_MASK_ODR         (0b11111)       // FILTER Output data rate setting

#define AD_BITS_EFILT_UNDEF (0b000 << 8)    // FILTER Enhanced 50/60 Hz filter undefined mode, use as OFF mode
#define AD_BITS_EFILT_MODE1 (0b010 << 8)    // FILTER Enhanced 50/60 Hz filter mode with Output Data Rate ODR = 27.27 SPS, settling time = 36.7 ms
#define AD_BITS_EFILT_MODE2 (0b011 << 8)    // FILTER Enhanced 50/60 Hz filter mode with Output Data Rate ODR = 25 SPS, settling time = 40 ms
#define AD_BITS_EFILT_MODE3 (0b100 << 8)    // FILTER Enhanced 50/60 Hz filter mode with Output Data Rate ODR = 20.67 SPS, settling time = 48.4 ms
#define AD_BITS_EFILT_MODE4 (0b101 << 8)    // FILTER Enhanced 50/60 Hz filter mode with Output Data Rate ODR = 20 SPS, settling time = 50 ms
#define AD_BITS_EFILT_MODE5 (0b110 << 8)    // FILTER Enhanced 50/60 Hz filter mode with Output Data Rate ODR = 16.67 SPS, settling time = 60 ms

#define AD_BITS_ORD_SINC51  (0b00 << 5)     // FILTER Order Sinc5 + Sinc1 (fast settling filter)
#define AD_BITS_ORD_SINC3   (0b11 << 5)     // FILTER Order Sinc3

#define AD_BITS_ODR_125K    0b00000         // FILTER Output Data Rate (Sinc3 = 125.0 kSPS,     Sinc5+Sinc1 = 125.0 kSPS)
#define AD_BITS_ODR_62K5    0b00010         // FILTER Output Data Rate (Sinc3 = 62.5 kSPS,      Sinc5+Sinc1 = 62.5 kSPS)
#define AD_BITS_ODR_31K2    0b00100         // FILTER Output Data Rate (Sinc3 = 31.25 kSPS,     Sinc5+Sinc1 = 31.25 kSPS)
#define AD_BITS_ODR_25K     0b00101         // FILTER Output Data Rate (Sinc3 = 25.0 kSPS,      Sinc5+Sinc1 = 25.0 kSPS)
#define AD_BITS_ODR_15K6    0b00110         // FILTER Output Data Rate (Sinc3 = 15.625 kSPS,    Sinc5+Sinc1 = 15.625 kSPS)
#define AD_BITS_ODR_10K4    0b00111         // FILTER Output Data Rate (Sinc3 = 10.417 kSPS,    Sinc5+Sinc1 = 10.390 kSPS)
#define AD_BITS_ODR_5K      0b01000         // FILTER Output Data Rate (Sinc3 = 5.0 kSPS,       Sinc5+Sinc1 = 4.994 kSPS)
#define AD_BITS_ODR_2K5     0b01001         // FILTER Output Data Rate (Sinc3 = 2.5 kSPS,       Sinc5+Sinc1 = 2.498 kSPS)
#define AD_BITS_ODR_1K      0b01010         // FILTER Output Data Rate (Sinc3 = 1.0 kSPS,       Sinc5+Sinc1 = 1.0 kSPS)
#define AD_BITS_ODR_500     0b01011         // FILTER Output Data Rate (Sinc3 = 500 SPS,        Sinc5+Sinc1 = 500 SPS)
#define AD_BITS_ODR_400     0b01100         // FILTER Output Data Rate (Sinc3 = 400.6 SPS,      Sinc5+Sinc1 = 395.5 SPS)
#define AD_BITS_ODR_200     0b01101         // FILTER Output Data Rate (Sinc3 = 200 SPS,        Sinc5+Sinc1 = 200 SPS)
#define AD_BITS_ODR_100     0b01110         // FILTER Output Data Rate (Sinc3 = 100.2 SPS,      Sinc5+Sinc1 = 100.2 SPS)
#define AD_BITS_ODR_60      0b01111         // FILTER Output Data Rate (Sinc3 = 60 SPS,         Sinc5+Sinc1 = 59.87 SPS)
#define AD_BITS_ODR_50      0b10000         // FILTER Output Data Rate (Sinc3 = 50 SPS,         Sinc5+Sinc1 = 49.92 SPS)
#define AD_BITS_ODR_20      0b10001         // FILTER Output Data Rate (Sinc3 = 20.0 SPS,       Sinc5+Sinc1 = 20.0 SPS)
#define AD_BITS_ODR_16      0b10010         // FILTER Output Data Rate (Sinc3 = 16.7 SPS,       Sinc5+Sinc1 = 16.7 SPS)
#define AD_BITS_ODR_10      0b10011         // FILTER Output Data Rate (Sinc3 = 10.0 SPS,       Sinc5+Sinc1 = 10.0 SPS)
#define AD_BITS_ODR_5       0b10100         // FILTER Output Data Rate (Sinc3 = 5.0 SPS,        Sinc5+Sinc1 = 5.0 SPS)

// ADC_GPIO_CONFIG Register
#define AD_MASK_SYNC_PIN_EN (0b1 << 11)     // ADC_GPIO_CONFIG Sync pin enable
#define AD_MASK_ERR_PIN_EN  (0b11 << 9)     // ADC_GPIO_CONFIG Error pin mode select

#define AD_BITS_ERR_PIN_OFF (0b00 << 9)     // ADC_GPIO_CONFIG Error pin disabled mode
#define AD_BITS_ERR_PIN_IN  (0b01 << 9)     // ADC_GPIO_CONFIG Error pin input mode (see datasheet for details)
#define AD_BITS_ERR_PIN_OUT (0b10 << 9)     // ADC_GPIO_CONFIG Error pin output mode (open drain, low on any non-masked error)