/*
 * tcs3472.h
 *
 *  Created on: 3 Jan 2018
 *      Author: Marcus
 */

#ifndef TCS3472_H_
#define TCS3472_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TCS3472_ADDR_ENABLE  0x00
#define TCS3472_ADDR_ATIME   0x01
#define TCS3472_ADDR_WTIME   0x03
#define TCS3472_ADDR_AILTL   0x04
#define TCS3472_ADDR_AILTH   0x05
#define TCS3472_ADDR_AIHTL   0x06
#define TCS3472_ADDR_AIHTH   0x07
#define TCS3472_ADDR_PERS    0x0c
#define TCS3472_ADDR_CONFIG  0x0d
#define TCS3472_ADDR_CONTROL 0x0f
#define TCS3472_ADDR_ID      0x12
#define TCS3472_ADDR_STATUS  0x13
#define TCS3472_ADDR_CDATAL  0x14
#define TCS3472_ADDR_CDATAH  0x15
#define TCS3472_ADDR_RDATAL  0x16
#define TCS3472_ADDR_RDATAH  0x17
#define TCS3472_ADDR_GDATAL  0x18
#define TCS3472_ADDR_GDATAH  0x19
#define TCS3472_ADDR_BDATAL  0x1a
#define TCS3472_ADDR_BDATAH  0x1b

#define TCS3472_REG_CMD_BIT 0x80

#define TCS3472_REG_CMD_TYPE_BYTE     (0b00 << 5)
#define TCS3472_REG_CMD_TYPE_AUTO_INC (0b01 << 5)
#define TCS3472_REG_CMD_TYPE_SF       (0b11 << 5)

#define TCS3472_REG_ENABLE_AIEN 0x10
#define TCS3472_REG_ENABLE_WEN  0x08
#define TCS3472_REG_ENABLE_AEN  0x02
#define TCS3472_REG_ENABLE_PON  0x01

#define TCS3472_REG_CONFIG_WLONG 0x02

#define TCS3472_REG_ID_34721_34725  0x44
#define TCS3472_REG_ID_34723_34727  0x4d

#define TCS3472_REG_STATUS_AVALID 0x01
#define TCS3472_REG_STATUS_AINT   0x10

#define TCS34721_I2C_ADDRESS (0x39 << 1)
#define TCS34723_I2C_ADDRESS (0x39 << 1)
#define TCS34725_I2C_ADDRESS (0x29 << 1)
#define TCS34727_I2C_ADDRESS (0x29 << 1)

typedef struct
{
    uint8_t   i2c_dev_addr;                /*!< i2c device address user specific field */
    uint16_t  comms_speed_khz;             /*!< Comms speed [kHz] : typically 400kHz for I2C           */
} TCS3472_Dev_t;

typedef enum
{
	TCS3472_REG_CONTROL_AGAIN_1X  = 0x00,
	TCS3472_REG_CONTROL_AGAIN_4X  = 0x01,
	TCS3472_REG_CONTROL_AGAIN_16X = 0x02,
	TCS3472_REG_CONTROL_AGAIN_60X = 0x03
} TCS3472_Gain_t;

typedef struct
{
	uint16_t c;
	uint16_t r;
	uint16_t g;
	uint16_t b;
} TCS3472_Color_t;

uint32_t TCS3472_setup(TCS3472_Dev_t * Dev, uint16_t speed, uint8_t addr);
uint32_t TCS3472_readId(TCS3472_Dev_t * Dev, uint8_t * id);
uint32_t TCS3472_enable(TCS3472_Dev_t * Dev);
uint32_t TCS3472_disable(TCS3472_Dev_t * Dev);
uint32_t TCS3472_setIntegrationTime(TCS3472_Dev_t * Dev, uint8_t value);
uint32_t TCS3472_setGain(TCS3472_Dev_t * Dev, TCS3472_Gain_t value);
uint32_t TCS3472_readColour(TCS3472_Dev_t * Dev, TCS3472_Color_t * colourData);


#ifdef __cplusplus
}
#endif

#endif /* TCS3472_H_ */
