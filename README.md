# IMU-sensor-fusion-ICM42670P
A project to demonstrate 6-axis IMU sensor fusion with ICM42670-P sensor

![imu](https://github.com/user-attachments/assets/049a680f-eb43-412c-83e5-cae4f33fadf2)

## Hardware
The IMU sensor used in ICM42670-P and the microcontroller is an XMC1302 series MCU with 16KB flash and 16KB ram. The board is powered by an external benchtop power supply and an on-board voltage regulator is used to generate 3.3V

## software
MCU and the IMU sensor communicate through I2C. The Mahony filter is implemented using fixed point arithmetic (no floating-point operations). The algorithm runs at 500 Hz that is triggered by the timer interrupt.  
