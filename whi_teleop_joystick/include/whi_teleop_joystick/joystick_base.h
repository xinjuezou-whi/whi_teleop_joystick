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

class JoystickBase
{
public:
	JoystickBase() = delete;
	JoystickBase(std::shared_ptr<ros::NodeHandle>& NodeHandle, const std::string& DataTopic = "")
		: node_handle_(NodeHandle), fdata_topic_(DataTopic) {};
	virtual ~BetteryBase() = default;

public:
	virtual void request() = 0;
	virtual void publish(void* Data) = 0;

protected:
	std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	std::string data_topic_{ "battery_data" };
	std::unique_ptr<ros::Publisher> pub_data_{ nullptr };
};
