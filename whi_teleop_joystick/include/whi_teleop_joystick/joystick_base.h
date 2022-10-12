/******************************************************************
joystick base for abstract interface

Features:
- abstract joystick operation interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-11: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class JoystickBase
{
public:
	JoystickBase() = delete;
	JoystickBase(std::shared_ptr<ros::NodeHandle>& NodeHandle,
		const std::string& Name = "", const std::string& DataTopic = "")
		: node_handle_(NodeHandle), device_name_(Name), data_topic_(DataTopic)
	{
		if (node_handle_)
		{
			pub_data_ = std::make_unique<ros::Publisher>(node_handle_->advertise<geometry_msgs::Twist>("cmd_vel", 50));
		}
	};
	virtual ~JoystickBase() = default;

public:
	void setRanges(const std::array<uint16_t, 2>& Ranges)
	{
		ranges_ = Ranges;
	}
	void setKinematicsLimits(double MaxLinear, double MaxAngular)
	{
		max_linear_ = MaxLinear;
		max_angular_ = MaxAngular;
	};
	virtual void fire() = 0;
	virtual void publish() = 0;

protected:
	std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	std::string device_name_;
	std::string data_topic_{ "joystick_data" };
	std::unique_ptr<ros::Publisher> pub_data_{ nullptr };
	double max_linear_{ 0.3 }; // m/s
	double max_angular_{ 1.0 }; // rad/s
	std::array<uint16_t, 2> pose_;
	std::array<uint16_t, 2> ranges_;
	double boundary_{ 1.0 };
};
