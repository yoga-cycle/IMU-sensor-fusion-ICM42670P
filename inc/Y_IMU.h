#pragma once
// #include <inc/HW_MPU6050.h>
#include "inc/HW_ICM42670.h"
#include <inc/Y_math.h>
#include <stdint.h>

// constants for 2000 DPS full scale
#define Y_IMU_GYRO_DPS_SCALER 125
#define Y_IMU_GYRO_DPS_DIVIDER 11

// constants for delta T sampling time
#define Y_IMU_DELTA_T_SCALER 1
#define Y_IMU_DELTA_T_DIVIDER 9

// Kp, KI
#define Y_IMU_KP 8
#define Y_IMU_KI 4
#define Y_IMU_INTEGRAL_SCALER 0
#define Y_IMU_INTEGRAL_MAX (1 << 28)

void Y_IMU_run_IMU();

void Y_IMU_run_IMU_2();

void Y_IMU_get_orientation_quaternion(int32_t result[4]);

void Y_IMU_get_gravity_measured(int32_t result[3]);

void Y_IMU_get_gravity_predicted(int32_t result[3]);

