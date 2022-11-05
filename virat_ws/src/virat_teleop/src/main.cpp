#include <iostream>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "io.hpp"

#define CONV_FACTOR 0.5

const std::string info_msg =
	"Reading from the keyboard and publishing to /cmd_vel\n\
---------------------------\n\
Moving around:\n\
        w\n\
    a   s   d\n\
\n\
Anything else : stop\n\
\n\
CTRL+C followed by ENTER to quit\n\
";

std::map<char, std::array<int, 4>> move_bindings = {
	{'w', {1, 0, 0, 0}},
	{'a', {0, 0, 0, 1}},
	{'d', {0, 0, 0, -1}},
	{'s', {-1, 0, 0, 0}},
};

geometry_msgs::Twist add(const geometry_msgs::Twist &original_twist, const std::array<int, 4> &move_modifier);

void initialize_twist(geometry_msgs::Twist &vel)
{
	vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "virat_teleop");

	ros::NodeHandle teleop_nh;

	ros::Publisher pub = teleop_nh.advertise<geometry_msgs::Twist>("/virat/cmd_vel", 10);

	geometry_msgs::Twist pub_vel;
	ros::Rate rate(100);

	char opcode = 0;

	initialize_twist(pub_vel);

	std::cout << info_msg << std::endl;

	while (ros::ok())
	{
		opcode = get_invisible_ch();

		if (opcode == 3) // CTRL+C
		{
			return 0;
		}

		if (move_bindings.count(opcode))
		{
			pub_vel = add(pub_vel, move_bindings[opcode]);
		}
		else
		{
			initialize_twist(pub_vel);
		}

		pub.publish(pub_vel);

		ros::spinOnce();
		rate.sleep();
	}
}

geometry_msgs::Twist add(const geometry_msgs::Twist &original_twist, const std::array<int, 4> &move_modifier)
{
	geometry_msgs::Twist result;

	result = original_twist;

	result.linear.x += CONV_FACTOR * move_modifier[0];
	result.linear.y += CONV_FACTOR * move_modifier[1];
	result.linear.z += CONV_FACTOR * move_modifier[2];
	result.angular.z += CONV_FACTOR * move_modifier[3];

	return result;
}