/******************************************************************
joystick driver instance for serial module

Features:
- joystick operation logic for serial hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-serial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_teleop_joystick/joystick_serial.h"

JoystickSerial::JoystickSerial(std::shared_ptr<ros::NodeHandle>& NodeHandle,
	const std::string& Name, const std::string& Topic, int DeviceId, uint8_t DataHead,
	const std::string& Port, unsigned int Baudrate/* = 9600*/)
	: JoystickBase(NodeHandle, Name, Topic)
	, serial_port_(Port), baudrate_(Baudrate), device_id_(DeviceId), pack_head_(DataHead)
	, pack_length_(DeviceId < 0 ? 9 : 10)
{
	// serial
	try
	{
		serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	}
	catch (serial::IOException& e)
	{
		ROS_FATAL_STREAM_NAMED("failed to open serial %s", serial_port_.c_str());
	}

	boundary_ = pow(0xfa0 - offset_, 3.0);
}

JoystickSerial:: ~JoystickSerial()
{
	terminated_.store(true);
	th_read_.join();

	if (serial_inst_)
	{
		serial_inst_->close();
	}
}

void JoystickSerial::fire()
{
	if (serial_inst_)
	{
		// spawn the read thread
		th_read_ = std::thread(std::bind(&JoystickSerial::threadReadSerial, this));
	}
}

void JoystickSerial::publish()
{
	if (pub_data_ && mtx_.try_lock())
	{
		geometry_msgs::Twist messageCmd;
		// linear
		messageCmd.linear.x = max_linear_ * pow(pose_[1] - offset_, 3.0) / boundary_;
#ifndef DEBUG
		std::cout << "linear: " << std::to_string(messageCmd.linear.x) << std::endl;
#endif
		pub_data_->publish(messageCmd);

		mtx_.unlock();
	}
}

void JoystickSerial::sendCommand(uint8_t Id, uint8_t Len, uint8_t* Data)
{

}

void JoystickSerial::fetchData(unsigned char* Data, size_t Length)
{
	unsigned char* head = Data;
	int offset = pack_length_ == 10 ? 1 : 0;
	bool idMatched = offset == 0 ? true : (head[offset] == device_id_);
	while (Length >= pack_length_)
	{
		if (head[0] != pack_head_ && idMatched)
		{
			head++;
			continue;
		}

		mtx_.lock();
		for (std::size_t i = 0; i < pose_.size(); ++i)
		{
			pose_[i] = uint16_t((head[(i << 1) + 1 + offset] << 8) | head[(i << 1) + 2 + offset]);
		}
		mtx_.unlock();

		Length -= pack_length_;
		head += pack_length_;
	}
}

void JoystickSerial::threadReadSerial()
{
	while (!terminated_.load())
	{
		if (serial_inst_)
		{
			size_t count = serial_inst_->available();
			if (count > 0)
			{
				unsigned char rbuff[count];
				size_t readNum = serial_inst_->read(rbuff, count);
				fetchData(rbuff, readNum);
			}
		}
	}
}
