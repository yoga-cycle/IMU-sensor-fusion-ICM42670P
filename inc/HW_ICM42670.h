/*
 * HW_ICM42670.h
 *
 *  Created on: Jan 7, 2026
 *      Author: user
 */

#ifndef INC_HW_ICM42670_H_
#define INC_HW_ICM42670_H_

#include <stdint.h>
#include "I2C_MASTER/i2c_master.h"

#define ICM42670_DEVICE_ADDRESS (0x68 << 1)
#define ICM42670_WHO_AM_I_DEFAULT 0x67

#define ICM42670_REG_MCLK_CONF              0x00
#define ICM42670_REG_DEVICE_CONFIG          0x01
#define ICM42670_REG_SIGNAL_PATH_RESET      0x02
#define ICM42670_REG_DRIVE_CONFIG           0x03
#define ICM42670_REG_INT_CONFIG             0x04
#define ICM42670_REG_TEMP_DATA1             0x09
#define ICM42670_REG_TEMP_DATA0             0x0A
#define ICM42670_REG_ACCEL_DATA_X1          0x0B
#define ICM42670_REG_ACCEL_DATA_X0          0x0C
#define ICM42670_REG_ACCEL_DATA_Y1          0x0D
#define ICM42670_REG_ACCEL_DATA_Y0          0x0E
#define ICM42670_REG_ACCEL_DATA_Z1          0x0F
#define ICM42670_REG_ACCEL_DATA_Z0          0x10
#define ICM42670_REG_GYRO_DATA_X1           0x11
#define ICM42670_REG_GYRO_DATA_X0           0x12
#define ICM42670_REG_GYRO_DATA_Y1           0x13
#define ICM42670_REG_GYRO_DATA_Y0           0x14
#define ICM42670_REG_GYRO_DATA_Z1           0x15
#define ICM42670_REG_GYRO_DATA_Z0           0x16
#define ICM42670_REG_TMST_FSYNCH1           0x17
#define ICM42670_REG_TMST_FSYNCH0           0x18
#define ICM42670_REG_APEX_DATA4             0x1D
#define ICM42670_REG_APEX_DATA5             0x1E
#define ICM42670_REG_PWR_MGMT0              0x1F
#define ICM42670_REG_GYRO_CONFIG0           0x20
#define ICM42670_REG_ACCEL_CONFIG0          0x21
#define ICM42670_REG_TEMP_CONFIG0           0x22
#define ICM42670_REG_GYRO_CONFIG1           0x23
#define ICM42670_REG_ACCEL_CONFIG1          0x24
#define ICM42670_REG_APEX_CONFIG0           0x25
#define ICM42670_REG_APEX_CONFIG1           0x26
#define ICM42670_REG_INT_SOURCE0            0x27
#define ICM42670_REG_INT_SOURCE1            0x28
#define ICM42670_REG_INT_SOURCE3            0x2A
#define ICM42670_REG_INT_SOURCE4            0x2B
#define ICM42670_REG_FIFO_LOST_PKT0         0x2C
#define ICM42670_REG_FIFO_LOST_PKT1         0x2D
#define ICM42670_REG_FIFO_COUNT1            0x2E
#define ICM42670_REG_FIFO_COUNT0            0x2F
#define ICM42670_REG_FIFO_DATA              0x30
#define ICM42670_REG_INT_STATUS             0x36
#define ICM42670_REG_INT_STATUS2            0x37
#define ICM42670_REG_INT_STATUS3            0x38
#define ICM42670_REG_SIGNAL_PATH_RESET_B0   0x4B
#define ICM42670_REG_INTF_CONFIG0           0x4C
#define ICM42670_REG_INTF_CONFIG1           0x4D
#define ICM42670_REG_PWR_MGMT_MCLK          0x4E
#define ICM42670_REG_INT_CONFIG0            0x4F
#define ICM42670_REG_INT_CONFIG1            0x50
#define ICM42670_REG_WHO_AM_I               0x75
#define ICM42670_REG_REG_BANK_SEL           0x76

/*
 * Scans for MPU6050 and return 1 if successful. else return 0.
 */
uint8_t HW_ICM42670_scan();

/*
 * initialize MPU6050 and return 1 if successful. else return 0.
 */
uint8_t HW_ICM42670_init();

/*
 * read acceleration from MPU6050 to input array
 */
uint8_t HW_ICM42670_get_acceleration_gyroscope_readings(int32_t accel_vector[3], int32_t gyro_vector[3]);

#endif /* INC_HW_ICM42670_H_ */
