/*
 * main.c
 *
 *  Created on: 2024 Apr 27 19:16:59
 *  Author: user
 */

#include <inc/Y_UART.h>
#include "DAVE.h"                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include "inc/Y_IMU.h"
/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           /* Initialization of DAVE APPs  */

  if (status != DAVE_STATUS_SUCCESS)
  {
    /* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  /* Placeholder for user application code. The while loop below can be replaced with user application code. */

  // debug variable to check for I2C device connection
  uint8_t success = 0;

  success = HW_ICM42670_scan();
  success = HW_ICM42670_init();

  int32_t orientation_quaternion[4];

  while(1U)
  {
	  // some delay so that animation code can update the frames
	  for(int32_t i = 0; i<20000; i++);

	  Y_IMU_get_orientation_quaternion(orientation_quaternion);

	  UART2_reset_buffer();

	  UART2_add_to_buffer(orientation_quaternion[3]);
	  UART2_add_to_buffer(orientation_quaternion[2]);
	  UART2_add_to_buffer(orientation_quaternion[1]);
	  UART2_add_to_buffer(orientation_quaternion[0]);

	  UART2_transmit_buffer();
  }
}

void IMU_timer_handler(void)
{
	// to check execution time
	DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_0);

	Y_IMU_run_IMU_2();

	// to check execution time
	DIGITAL_IO_SetOutputLow(&DIGITAL_IO_0);
}
