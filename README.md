# nRF52_TCS34725
Nordic Semiconductor nRF52 // TCS34725 Colour Sensor Driver

## About ##
Provides basic firmware support for the TCS34725 RGB Sensor

Communication with the chip is handled via I2C and this driver uses the TWI hardware peripheral on the nRF52; requiring that the relevant flags in sdk_config.h are also set.

Tested with nRF5_SDK_13.0.0
