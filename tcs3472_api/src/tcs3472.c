/*
 * tcs3472.c
 *
 *  Created on: 3 Jan 2018
 *      Author: Marcus
 */
#include "sam_block.h" /* missing the header file on src folder */

#include "tcs3472.h"

#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"

/**
 * @brief TWI master instance
 *
 * Instance of TWI master driver that would be used for communication with VL53L0X.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);

static uint32_t TCS3472_readByte(TCS3472_Dev_t * Dev, uint8_t reg, uint8_t * p_data);
static uint32_t TCS3472_readWord(TCS3472_Dev_t * Dev, uint8_t reg, uint16_t * p_data);
static uint32_t TCS3472_writeByte(TCS3472_Dev_t * Dev, uint8_t reg, uint8_t value);

uint32_t TCS3472_readId(TCS3472_Dev_t * Dev, uint8_t * id)
{
	return TCS3472_readByte(Dev, TCS3472_ADDR_ID, id);
}

static uint32_t TCS3472_init(TCS3472_Dev_t * Dev)
{
	uint32_t nrf_speed;
	ret_code_t ret;
	uint8_t id;

	if(Dev->comms_speed_khz == 400){
		nrf_speed = NRF_TWI_FREQ_400K;
	} else if(Dev->comms_speed_khz == 250){
		nrf_speed = NRF_TWI_FREQ_250K;
	} else if(Dev->comms_speed_khz == 100){
		nrf_speed = NRF_TWI_FREQ_100K;
	} else {
		NRF_LOG_ERROR("Invalid TWI comms speed.");
		return 1;
	}

	const nrf_drv_twi_config_t config =
	{
	   .scl                = SENSOR_SCL_PIN,
	   .sda                = SENSOR_SDA_PIN,
	   .frequency          = nrf_speed,
	   .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	   .clear_bus_init     = false
	};

	ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

	if (NRF_SUCCESS == ret)
	{
		nrf_drv_twi_enable(&m_twi_master);

		NRF_LOG_DEBUG("TWI init successful\r\n");

		ret = TCS3472_readId(Dev, &id);
		if (ret != NRF_SUCCESS)
		{
			return ret;
		}

		if (id != TCS3472_REG_ID_34721_34725 && id != TCS3472_REG_ID_34723_34727)
		{
			NRF_LOG_DEBUG("invalid id: %d\r\n", id);
			return NRF_ERROR_NOT_FOUND;
		}
		else
		{
			NRF_LOG_DEBUG("valid id\r\n");
		}

	} else {
		NRF_LOG_ERROR("TWI init failed\r\n");
	}

	return ret;
}

uint32_t TCS3472_setup(TCS3472_Dev_t * Dev, uint16_t speed, uint8_t addr)
{
	ret_code_t ret;

	Dev->comms_speed_khz = speed;
	Dev->i2c_dev_addr    = addr;

	ret = TCS3472_init(Dev);

	return ret;
}

uint32_t TCS3472_enable(TCS3472_Dev_t * Dev)
{
	ret_code_t ret;

	ret = TCS3472_writeByte(Dev, TCS3472_ADDR_ENABLE, TCS3472_REG_ENABLE_PON);
	if (ret != NRF_SUCCESS)
	{
	   return ret;
	}
	//nrf_delay_ms(3);
	ret = TCS3472_writeByte(Dev, TCS3472_ADDR_ENABLE, TCS3472_REG_ENABLE_PON | TCS3472_REG_ENABLE_AEN);
	return ret;
}

uint32_t TCS3472_disable(TCS3472_Dev_t * Dev)
{
	ret_code_t ret;
	uint8_t value;

	ret = TCS3472_readByte(Dev, TCS3472_ADDR_ENABLE, &value);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }

    value &= ~(TCS3472_REG_ENABLE_PON | TCS3472_REG_ENABLE_AEN);

	ret = TCS3472_writeByte(Dev, TCS3472_ADDR_ENABLE, value);
	return ret;
}

uint32_t TCS3472_setIntegrationTime(TCS3472_Dev_t * Dev, uint8_t value)
{
	return TCS3472_writeByte(Dev, TCS3472_ADDR_ATIME, value);
}

uint32_t TCS3472_setGain(TCS3472_Dev_t * Dev, TCS3472_Gain_t value)
{
	return TCS3472_writeByte(Dev, TCS3472_ADDR_CONTROL, (uint8_t)value);
}

uint32_t TCS3472_readColour(TCS3472_Dev_t * Dev, TCS3472_Color_t * colourData)
{
	ret_code_t ret;

	ret = TCS3472_readWord(Dev, TCS3472_ADDR_CDATAL, &colourData->c);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }
	ret = TCS3472_readWord(Dev, TCS3472_ADDR_RDATAL, &colourData->r);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }
	ret = TCS3472_readWord(Dev, TCS3472_ADDR_GDATAL, &colourData->g);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }
	ret = TCS3472_readWord(Dev, TCS3472_ADDR_BDATAL, &colourData->b);
	return ret;
}

static uint32_t TCS3472_readByte(TCS3472_Dev_t * Dev, uint8_t reg, uint8_t * p_data)
{
	ret_code_t ret;

	uint8_t cmd = TCS3472_REG_CMD_BIT | TCS3472_REG_CMD_TYPE_BYTE | reg;

	ret = nrf_drv_twi_tx(&m_twi_master, Dev->i2c_dev_addr >> 1, &cmd, 1, true);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }

    ret = nrf_drv_twi_rx(&m_twi_master, Dev->i2c_dev_addr >> 1, p_data, 1);
    if (NRF_SUCCESS != ret)
    {
 	   return ret;
    }

    return ret;
}

static uint32_t TCS3472_readWord(TCS3472_Dev_t * Dev, uint8_t reg, uint16_t * p_data)
{
	ret_code_t ret;
    uint8_t temp[2];

	uint8_t cmd = TCS3472_REG_CMD_BIT | TCS3472_REG_CMD_TYPE_BYTE | reg;

	ret = nrf_drv_twi_tx(&m_twi_master, Dev->i2c_dev_addr >> 1, &cmd, 1, true);
    if (ret != NRF_SUCCESS)
    {
 	   return ret;
    }

    ret = nrf_drv_twi_rx(&m_twi_master, Dev->i2c_dev_addr >> 1, temp, 2);
    if (NRF_SUCCESS != ret)
    {
 	   return ret;
    }

    *p_data = ((uint16_t)temp[1] << 8) | (uint16_t)temp[0];

	return ret;
}

static uint32_t TCS3472_writeByte(TCS3472_Dev_t * Dev, uint8_t reg, uint8_t value)
{
	ret_code_t ret;
	uint8_t data[2];

	data[0] = TCS3472_REG_CMD_BIT | TCS3472_REG_CMD_TYPE_BYTE | reg;
	data[1] = value;

	ret = nrf_drv_twi_tx(&m_twi_master, Dev->i2c_dev_addr >> 1, data, 2, true);

	return ret;
}
