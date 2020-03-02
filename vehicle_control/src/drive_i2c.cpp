#include <ros/ros.h>
#include "include/PCA9685.h"
#include <project_drive_msgs/CarParams.h>

#define I2C_ADDRESS 0x40
#define I2C_BUS 1


void driveCallback(const project_drive_msgs::CarParams::ConstPtr &msg) {
	PCA9685 *controller = new PCA9685(I2C_BUS,I2C_ADDRESS);
	// set up pwm frequency
	
	// convert angle and velocity to pwm values
	
	// send i2c command	
}

int main(int argc, char **argv){		
	ros::init(argc, argv, "drive_i2c");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("drive_params", 1000, driveCallback);

	ros::spin();
}
