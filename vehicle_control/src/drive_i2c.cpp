#include <ros/ros.h>
#include "include/PCA9685.h"
#include <project_drive_msgs/CarParams.h>

#define I2C_ADDRESS 0x40
#define I2C_BUS 1

#define ESC_CHANNEL 1
#define SERVO_CHANNEL 2

PCA9685 *controller = new PCA9685(I2C_BUS,I2C_ADDRESS);
double map(double input, double inrange_m, double inrange_M, double outrange_m, double outrange_M) {
	double output = (input - inrange_m)/(inrange_M - inrange_m) * (outrange_M - outrange_m) + outrange_m;
	return output;
}


void driveCallback(const project_drive_msgs::CarParams::ConstPtr &msg) {
	// convert angle and velocity to pwm values
	int servo_pulse_len = (int) map((double) msg->angle, 0.0, 180.0, 0.0, 4095.0);	
	int esc_pulse_len = (int) map((double) msg->velocity, -100.0, 100.0, 0.0, 4095.0);
	// send i2c command	
	controller->setPWM(SERVO_CHANNEL, servo_pulse_len);
	controller->setPWM(ESC_CHANNEL, esc_pulse_len);
}

int main(int argc, char **argv){		
	ros::init(argc, argv, "drive_i2c");
	ros::NodeHandle n;

	// set up pwm frequency
	controller->setPWMFreq(100);
	ros::Subscriber sub = n.subscribe("drive_params", 1000, driveCallback);

	ros::spin();
}
