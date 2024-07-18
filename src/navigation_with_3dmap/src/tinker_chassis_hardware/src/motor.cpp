#include "motor_interface/motor.hpp"

Motor::Motor(int _ID, const MotorPreset *_Preset, MotorParamter _Paramter)
{
	//Initialize Self
	ID = _ID;
	Preset = _Preset;
	Paramter = _Paramter;

	//Initialize Time
	last_looptime = rclcpp::Clock().now();

	//Initialize Setpoint
	Setpoint = 0;

	//Initialize PID
	Kp = 0;
	Ki = 0;
	Kd = 0;
	Kf = 0;
	KmaxI = 1000;
	KmaxO = 1.0;

	//Initialize PID Variables
	VError_Intergral = 0;

	//Calibrate
	isCalibrating = Paramter.DoCalibration;
	CalibrateDuration = 0;
	CalibrationValue = 0;
}

void Motor::setCoefficients(double _Kp, double _Ki, double _Kd, double _Kf, double _KmaxI, double _KmaxO)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	Kf = _Kf;
	KmaxI = _KmaxI;
	KmaxO = _KmaxO;
}

double Motor::getVelocity()
{
	return Hardware()->motors[ID].speed_rpm * Preset->RPMToRad;
}

double Motor::getPosition()
{
	return (Hardware()->motors[ID].total_angle * Preset->TickToRad) - CalibrationValue;
}

void Motor::update()
{

	//get Time
	double dt = (rclcpp::Clock().now() - last_looptime).seconds();
	last_looptime = rclcpp::Clock().now();

	if (!dt)
	{
		//Return at zero delta time
		return;
	}

	//PID Closeloop
	//Reference: https://bitbucket.org/AndyZe/pid
	double c_ = 1.0;

	//Calculate Error
	double real = (Paramter.CloseloopType == CLOSELOOP_VELOCITY) ? getVelocity() : getPosition();
	double set = Setpoint;

	double error = set - real;

	VError.value[2] = VError.value[1];
	VError.value[1] = VError.value[0];
	VError.value[0] = error;

	//intergrate error
	VError_Intergral += VError.value[0] * dt;
	if (VError_Intergral > KmaxI)
		VError_Intergral = KmaxI;
	if (VError_Intergral < -KmaxI)
		VError_Intergral = -KmaxI;

	VError_Filtered.value[2] = VError_Filtered.value[1];
	VError_Filtered.value[1] = VError_Filtered.value[0];
	VError_Filtered.value[0] = (1 / (1 + c_ * c_ + 1.414 * c_)) * (VError.value[2] + 2 * VError.value[1] + VError.value[0] -
																   (c_ * c_ - 1.414 * c_ + 1) * VError_Filtered.value[2] -
																   (-2 * c_ * c_ + 2) * VError_Filtered.value[1]);

	//calculate error derivative
	VError_Derivative.value[2] = VError_Derivative.value[1];
	VError_Derivative.value[1] = VError_Derivative.value[0];
	VError_Derivative.value[0] = (VError.value[0] - VError.value[1]) / dt;

	//filter error derivative
	VError_Derivative_Filtered.value[2] = VError_Derivative_Filtered.value[1];
	VError_Derivative_Filtered.value[1] = VError_Derivative_Filtered.value[0];

	VError_Derivative_Filtered.value[0] =
		(1 / (1 + c_ * c_ + 1.414 * c_)) *
		(VError_Derivative.value[2] + 2 * VError_Derivative.value[1] + VError_Derivative.value[0] -
		 (c_ * c_ - 1.414 * c_ + 1) * VError_Derivative_Filtered.value[2] - (-2 * c_ * c_ + 2) * VError_Derivative_Filtered.value[1]);

	//output power
	double output = Kp * VError_Filtered.value[0] + Ki * VError_Intergral + Kd * VError_Derivative_Filtered.value[0] + Kf;

	//limit output
	double output_sign = (output < 0.0) ? -1.0 : 1.0;
	output = output_sign * std::min( std::abs(output), KmaxO );

	//output pwm
	int pwm_max_value = Preset->PWMMaxValue;
	int out_power = (int)(output * pwm_max_value);

	if (out_power > pwm_max_value)
		out_power = pwm_max_value;
	if (out_power < -pwm_max_value)
		out_power = -pwm_max_value;
	// RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Motor output: %d", out_power);
	// RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "ID: %d", ID);
	Hardware()->motors[ID].power = out_power;
	Hardware()->update();
}