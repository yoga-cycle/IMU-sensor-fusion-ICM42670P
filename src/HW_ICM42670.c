/*
 * HW_ICM42670.c
 *
 *  Created on: Jan 7, 2026
 *      Author: user
 */
#include "inc/HW_ICM42670.h"

__attribute__((section(".ram_code")))
uint8_t HW_ICM42670_scan()
{
	I2C_MASTER_STATUS_t i2c_status;

	uint8_t who_am_i_address = ICM42670_REG_WHO_AM_I;
	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &who_am_i_address, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	uint8_t ICM42670_response = 0x00;
	i2c_status = I2C_MASTER_Receive(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &ICM42670_response, 1, true, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	/*
	 * Verify if response from sensor is as per datasheet
	 */
	if(ICM42670_response == ICM42670_WHO_AM_I_DEFAULT)
	{
		return 1;
	}

	return 0;
}


__attribute__((section(".ram_code")))
uint8_t HW_ICM42670_init()
{
	I2C_MASTER_STATUS_t i2c_status;

	/*
	 * first wake up sensor by writing to PWR_MGMT0
	 */
	uint8_t power_management_0_addr = ICM42670_REG_PWR_MGMT0;

	/* bit 7   - ACCEL_LP_CLK_SEL - 1
	 * bit 6:5 - RESERVED         - 0
	 * bit 4   - IDLE             - 1
	 * bit 3:2 - GYRO_MODE        - 11 (low noise mode)
	 * bit 1:0 - ACCEL_MODE       - 11 (low noise mode)
	 * value: 0b10011111 - 0x9f
	 */
	uint8_t power_management_0_value = 0x9f;

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &power_management_0_addr, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, false, ICM42670_DEVICE_ADDRESS, &power_management_0_value, 1, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	/*
	 * Some delay
	 */
	for(int32_t i=0xffff; i>0; i--);

	/* configure GYRO_CONFIG0 */
	uint8_t gyro_config_0_addr = ICM42670_REG_GYRO_CONFIG0;

	/*
	 * bit 7   - reserved       - 0
	 * bit 6:5 - GYRO_UI_FS_SEL - 00 (2000 dps)
	 * bit 4   - reserved       - 0
	 * bit 3:0 - GYRO_ODR       - 0110 (800 Hz)
	 * value: 0b00000110 - 0x06
	 */
	uint8_t gyro_config_0_value = 0x06;

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &gyro_config_0_addr, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, false, ICM42670_DEVICE_ADDRESS, &gyro_config_0_value, 1, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	/* configure GYRO_CONFIG1 */
	uint8_t gyro_config_1_addr = ICM42670_REG_GYRO_CONFIG1;

	/*
	 * bit 7:3 - reserved        - 00110
	 * bit 2:0 - GYRO_UI_FILT_BW - 001 (180 Hz)
	 * value: 0b00110001 - 0x31 (same as default value)
	 */
	uint8_t gyro_config_1_value = 0x31;

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &gyro_config_1_addr, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, false, ICM42670_DEVICE_ADDRESS, &gyro_config_1_value, 1, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	/* configure ACCEL_CONFIG0 */
	uint8_t accel_config_0_addr = ICM42670_REG_ACCEL_CONFIG0;

	/*
	 * bit 7   - reserved         - 0
	 * bit 6:5 - ACCEL_UI_FS_SEL  - 00 (16 g)
	 * bit 4   - reserved         - 0
	 * bit 3:0 - ACCEL_ODR        - 0110 (800 Hz)
	 * value: 0b00000110 - 0x06
	 */
	uint8_t accel_config_0_value = 0x06;

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &accel_config_0_addr, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, false, ICM42670_DEVICE_ADDRESS, &accel_config_0_value, 1, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	/* configure ACCEL_CONFIG1 */
	uint8_t accel_config_1_addr = ICM42670_REG_ACCEL_CONFIG1;

	/*
	 * bit 7   - reserved         - 0
	 * bit 6:4 - ACCEL_UI_AVG     - 000 // not used in LN mode
	 * bit 3   - reserved         - 0
	 * bit 2:0 - ACCEL_UI_FILT_BW - 001 (180 Hz)
	 * value: 0b00000001 - 0x01
	 */
	uint8_t accel_config_1_value = 0x1;

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &accel_config_1_addr, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, false, ICM42670_DEVICE_ADDRESS, &accel_config_1_value, 1, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	return 1;
}

__attribute__((section(".ram_code")))
uint8_t HW_ICM42670_get_acceleration_gyroscope_readings(int32_t accel_vector[3], int32_t gyro_vector[3])
{
	I2C_MASTER_STATUS_t i2c_status;
	uint8_t start_address = ICM42670_REG_ACCEL_DATA_X1;
	uint8_t raw_data[12] = {0};

	i2c_status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, &start_address, 1, false);
	if(i2c_status!=0U)
	{
		return 0;
	}

	i2c_status = I2C_MASTER_Receive(&I2C_MASTER_0, true, ICM42670_DEVICE_ADDRESS, raw_data, 12, true, true);
	if(i2c_status!=0U)
	{
		return 0;
	}

	accel_vector[0] = (int16_t)((raw_data[1]) | (raw_data[0] << 8));
	accel_vector[1] = (int16_t)((raw_data[3]) | (raw_data[2] << 8));
	accel_vector[2] = (int16_t)((raw_data[5]) | (raw_data[4] << 8));

	gyro_vector[0] = (int16_t)((raw_data[7]) | (raw_data[6] << 8));
	gyro_vector[1] = (int16_t)((raw_data[9]) | (raw_data[8] << 8));
	gyro_vector[2] = (int16_t)((raw_data[11]) | (raw_data[10] << 8));

	return 1;
}
