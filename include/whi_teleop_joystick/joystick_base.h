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
		const std::string& Name = "", const std::string& DataTopic = "cmd_vel")
		: node_handle_(NodeHandle), device_name_(Name), data_topic_(DataTopic)
	{
		if (node_handle_)
		{
			pub_data_ = std::make_unique<ros::Publisher>(node_handle_->advertise<geometry_msgs::Twist>(data_topic_, 50));
		}
	};
	virtual ~JoystickBase() = default;

public:
	void setRanges(const std::array<uint16_t, 2>& Ranges)
	{
		ranges_ = Ranges;
		for (std::size_t i = 0; i < std::min(boundaries_.size(), ranges_.size()); ++i)
		{
			boundaries_[i] = pow(ranges_[i], 3.0);
			boundaries_raw_[i] = ranges_[i];
		}
	}
	void setKinematicsLimits(double MaxLinear, double MaxAngular)
	{
		max_linear_ = MaxLinear;
		max_angular_ = MaxAngular;
	};
	using ButtonList = std::map<std::string, int>;
	void setButtons(const ButtonList& Buttons) { buttons_ = Buttons; };
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
	std::array<uint16_t, 2> ranges_{ 0xfa0 };
	std::array<double, 2> boundaries_{ 1.0, 1.0 };
	std::array<double, 2> boundaries_raw_{ 1.0, 1.0 };
	ButtonList buttons_;
	uint16_t button_triggered_{ 0 };
};
