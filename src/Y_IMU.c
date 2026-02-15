/*
 * Y-IMU.c
 *
 *  Created on: Nov 8, 2025
 *      Author: user
 */
#include "inc/Y_IMU.h"

/***** orientation output variables *****/
/****************************************/

// gravity vector in earth frame [0,0,1]
static int32_t gravity_vector[3] = {0,0,FIXED_POINT_UNIT_MAGNITUDE};

// orientation quaternion in fixed point representation
static int32_t orientaion_quaternion[4] = {FIXED_POINT_UNIT_MAGNITUDE,0,0,0};

// gravity vector in sensor frame [gx,gy,gz]
static int32_t gravity_measured_vector[3];

// gravity vector in sensor frame predicted using orientation quaternion
static int32_t gravity_predicted_vector[3];

/****************************************/
/***** orientation output variables *****/


/***** Mahony filter variables *****/
/***********************************/

// estimated gyro bias, i.e, output of mahony PI filter
static int32_t gyro_bias[3];

// instantaneous error in gyro bias estimation
static int32_t gyro_bias_error[3];

// integral of gyro bias estimation error
static int32_t gyro_bias_error_integral[3];

/***********************************/
/***** Mahony filter variables *****/

/*
 * converts the 3-axis gyroscope readings to axis-angle notation
 * The 3-axis gyro input is RAW readings from the sensor
 * The angle output is in degrees/second
 * the axis output is a unit vector in fixed-point notation
 * The function uses the scaling values Y_IMU_GYRO_DPS_SCALER, Y_IMU_GYRO_DPS_DIVIDER to convert
 * RAW readings to degree/second
 * */
__attribute__((section(".ram_code")))
static inline void Y_IMU_get_gyro_axis_angle(int32_t gyro[3], int32_t result[4])
{
	int32_t OmegaX, OmegaY, OmegaZ;

	OmegaX = gyro[0];
	OmegaY = gyro[1];
	OmegaZ = gyro[2];

	int32_t magnitude = 0;

	magnitude = Math_sqrt_of_sum_of_squares(OmegaX, OmegaY);
	magnitude = Math_sqrt_of_sum_of_squares(OmegaZ, magnitude);

	OmegaX = (OmegaX << FIXED_POINT_SCALE);
	OmegaY = (OmegaY << FIXED_POINT_SCALE);
	OmegaZ = (OmegaZ << FIXED_POINT_SCALE);

	OmegaX = OmegaX / magnitude;
	OmegaY = OmegaY / magnitude;
	OmegaZ = OmegaZ / magnitude;

	result[0] = (magnitude*Y_IMU_GYRO_DPS_SCALER) >> Y_IMU_GYRO_DPS_DIVIDER;
	result[1] = OmegaX;
	result[2] = OmegaY;
	result[3] = OmegaZ;
}

/*
 * converts the axis-angle gyroscope readings into a quaternion representing the rotation in one sampling time
 * quaternion = [cos(theta/2),sin(theta/2)*unit_vector[3]]
 * The sampling time used is defined as Y_IMU_DELTA_T_SCALER and Y_IMU_DELTA_T_DIVIDER
 * axis-angle input is in degress/second and unit vector in fixed point notation
 * quaternion output is normalized to unit magnitude in fixed point notation
 * */
__attribute__((section(".ram_code")))
static inline void Y_IMU_axis_angle_to_quaternion_2(int32_t axis_angle[4], int32_t result[4])
{
	int32_t sine, cosine;

	// convert DPS readings to degrees based on sample time delta T and scale by FIXED_POINT_SCALE
	int32_t degrees = ((axis_angle[0] << FIXED_POINT_SCALE)*Y_IMU_DELTA_T_SCALER) >> Y_IMU_DELTA_T_DIVIDER;
	int32_t half_angle = (degrees >> 1);
	int32_t radians = (half_angle*143) >> 13; // multiply by (3.14/180). (143/8192) ~ (3.14/180).note radian here is scaled by FIXED_POINT_SCALE

	// sin(x)-> x for x->0
	sine = radians;

	// cos(x) ~ 1-x^2/2
	int32_t x_2_by_2 = ((radians*radians) >> FIXED_POINT_SCALE) >> 1;
	cosine = (FIXED_POINT_UNIT_MAGNITUDE) - x_2_by_2;

	result[0] = cosine;
	result[1] = (axis_angle[1]*sine) >> FIXED_POINT_SCALE;
	result[2] = (axis_angle[2]*sine) >> FIXED_POINT_SCALE;
	result[3] = (axis_angle[3]*sine) >> FIXED_POINT_SCALE;
}

__attribute__((section(".ram_code")))
void Y_IMU_get_orientation_quaternion(int32_t result[4])
{
	result[0] = orientaion_quaternion[0];
	result[1] = orientaion_quaternion[1];
	result[2] = orientaion_quaternion[2];
	result[3] = orientaion_quaternion[3];
}

__attribute__((section(".ram_code")))
void Y_IMU_get_gravity_measured(int32_t result[3])
{
	result[0] = gravity_measured_vector[0];
	result[1] = gravity_measured_vector[1];
	result[2] = gravity_measured_vector[2];
}

__attribute__((section(".ram_code")))
void Y_IMU_get_gravity_predicted(int32_t result[3])
{
	result[0] = gravity_predicted_vector[0];
	result[1] = gravity_predicted_vector[1];
	result[2] = gravity_predicted_vector[2];
}

/*
 * function that runs IMU fusion. called at every timer interrupt [eg. 512 Hz]
 */
__attribute__((section(".ram_code")))
void Y_IMU_run_IMU_2()
{
	int32_t Omega[3];

	// read the sensor and get [gx,gy,gz] and [wx,wy,wz] (acceleromter and gyroscope readings in sensor frame)
	HW_ICM42670_get_acceleration_gyroscope_readings(gravity_measured_vector, Omega);

	// correct for gyro bias
	// this is sensor fusion step [also called correction step]
	Omega[0] += gyro_bias[0];
	Omega[1] += gyro_bias[1];
	Omega[2] += gyro_bias[2];

	int32_t gyro_axis_angle[4], gyro_quaternion[4];

	// represent gyroscope readings [wx,wy,wz] as a single axis and rotation speed
	Y_IMU_get_gyro_axis_angle(Omega, gyro_axis_angle);

	// convert the axis-angle rotation speed representation into a quaternion
	// rotation speed is converted into an angle based on sampling time
	Y_IMU_axis_angle_to_quaternion_2(gyro_axis_angle, gyro_quaternion); // gyro quaternion update

	// update the orientation quaternion using the gyro quaternion [ This is the prediction step]
	Math_Quaternion_multiply(orientaion_quaternion,gyro_quaternion,orientaion_quaternion);
	Math_Quaternion_normalize(orientaion_quaternion);

	// normalize the gravity vector in sensor frame
	Math_Vector_normalize(gravity_measured_vector);

	// rotate the gravity vector with the predicted orientation quaternion and store the result in gravity_predicted_vector
	Math_Quaternion_rotate_vector(orientaion_quaternion, gravity_vector, gravity_predicted_vector);

	// if orientation quaternion is correct, gravity_predicted_error and gravity_measured_vector will match and cross product is zero
	// else, the orientation quaternion is incorrect due to gyro bias. The value of error is the cross-product vector.
	// this cross product vector is fed into the PI controller to reduce the error to zero.
	Math_Vector_cross_product(gravity_measured_vector, gravity_predicted_vector, gyro_bias_error);

	/* calculate the correction value to implement in the correction step
	 * by using the PI compensator
	 */

	/*
	 * update the gyro_bias_integral_error
	 */
	gyro_bias_error_integral[0] += gyro_bias_error[0];
	gyro_bias_error_integral[1] += gyro_bias_error[1];
	gyro_bias_error_integral[2] += gyro_bias_error[2];

	/*
	 * If Y_IMU_INTEGRAL_MAX is reached, prevent further increase in integral error
	 * this acts as anti-windup
	 */
	if(gyro_bias_error_integral[0] > Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[0] = Y_IMU_INTEGRAL_MAX;
	}
	if(gyro_bias_error_integral[0] < -Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[0] = -Y_IMU_INTEGRAL_MAX;
	}

	if(gyro_bias_error_integral[1] > Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[1] = Y_IMU_INTEGRAL_MAX;
	}
	if(gyro_bias_error_integral[1] < -Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[1] = -Y_IMU_INTEGRAL_MAX;
	}

	if(gyro_bias_error_integral[2] > Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[2] = Y_IMU_INTEGRAL_MAX;
	}
	if(gyro_bias_error_integral[2] < -Y_IMU_INTEGRAL_MAX)
	{
		gyro_bias_error_integral[2] = -Y_IMU_INTEGRAL_MAX;
	}

	/*
	 * calculate output of PI controller and update the gyro_bias value
	 */
	gyro_bias[0] = (Y_IMU_KP * gyro_bias_error[0]) +
			       ((Y_IMU_KI * ((gyro_bias_error_integral[0] * Y_IMU_DELTA_T_SCALER) >> Y_IMU_DELTA_T_DIVIDER)) >> Y_IMU_INTEGRAL_SCALER);
	gyro_bias[1] = (Y_IMU_KP * gyro_bias_error[1]) +
			       ((Y_IMU_KI * ((gyro_bias_error_integral[1] * Y_IMU_DELTA_T_SCALER) >> Y_IMU_DELTA_T_DIVIDER)) >> Y_IMU_INTEGRAL_SCALER);
	gyro_bias[2] = (Y_IMU_KP * gyro_bias_error[2]) +
			       ((Y_IMU_KI * ((gyro_bias_error_integral[2] * Y_IMU_DELTA_T_SCALER) >> Y_IMU_DELTA_T_DIVIDER)) >> Y_IMU_INTEGRAL_SCALER);

	/*
	 * End of algorithm. The gyro_bias calculated is used in next iteration to compensate for gyroscope readings
	 */
}
