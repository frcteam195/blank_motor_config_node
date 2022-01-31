#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Config.h"
#include "rio_control_node/Motor_Configuration.h"

#include <thread>
#include <string>
#include <mutex>

ros::NodeHandle* node;

static std::vector<int> motor_ids;
static bool enabled;

static rio_control_node::Motor_Configuration config_msg;
static rio_control_node::Motor_Control control_msg;

static ros::Publisher control_publisher;
static ros::Publisher config_publisher;

void init_messages()
{
	static bool init_complete = false;

	if(init_complete)
		return;


	for(std::vector<int>::iterator i = motor_ids.begin();
		i != motor_ids.end();
		i++)
	{
		ROS_INFO("Publishing blank data for motor %d", (*i));

		rio_control_node::Motor_Config motor_config;
		motor_config.id = (*i);
		motor_config.controller_type = rio_control_node::Motor_Config::TALON_FX;
		motor_config.controller_mode = rio_control_node::Motor_Config::MASTER;
		motor_config.invert_type = rio_control_node::Motor_Config::NONE;
		if(motor_config.id == 14)
		{
			motor_config.controller_mode = rio_control_node::Motor_Config::FOLLOWER;
			motor_config.invert_type = rio_control_node::Motor_Config::OPPOSE_MASTER;
		}
		if(motor_config.id == 13)
		{
			motor_config.closed_loop_ramp = 2.5;
			motor_config.peak_output_reverse = 0.3;
			motor_config.kP = 0.03;
			motor_config.kD = 0.04;
			motor_config.kF = 0.047651;
		}
		motor_config.neutral_mode = rio_control_node::Motor_Config::COAST;
		motor_config.voltage_compensation_saturation = 12.0;
		motor_config.voltage_compensation_enabled = true;

		config_msg.motors.push_back(motor_config);

		rio_control_node::Motor motor_control;
		motor_control.id = (*i);
		motor_control.controller_type = rio_control_node::Motor::TALON_FX;
		motor_control.output_value = 0;
		if(motor_control.id == 8 || motor_control.id == 12)
		{
			motor_control.control_mode = rio_control_node::Motor::POSITION;
		}
		else if(motor_control.id == 13)
		{
			motor_control.control_mode = rio_control_node::Motor::VELOCITY;
		}
		else if(motor_control.id == 14)
		{
			motor_control.control_mode = rio_control_node::Motor::FOLLOWER;
			motor_control.output_value = 13;
		}
		else
		{
			motor_control.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
		}
		

		control_msg.motors.push_back(motor_control);
	}

	init_complete = true;
}

void motorStatusCallback(const rio_control_node::Motor_Status& msg)
{
	(void) msg;

	if (enabled && motor_ids.size() > 0)
	{
		init_messages();

		static int config_counter = 0;

		control_publisher.publish(control_msg);

		if(config_counter % 100 == 0)
		{
			config_publisher.publish(config_msg);
		}

		config_counter ++;
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "blank_motor_config_node");

	ros::NodeHandle n;

	node = &n;

	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 10, motorStatusCallback);

	control_publisher = node->advertise<rio_control_node::Motor_Control>("MotorControl", 1);
	config_publisher = node->advertise<rio_control_node::Motor_Configuration>("MotorConfiguration", 1);

	enabled = node->getParam("/blank_motor_config_node/motor_ids", motor_ids);

	ros::spin();
	return 0;
}