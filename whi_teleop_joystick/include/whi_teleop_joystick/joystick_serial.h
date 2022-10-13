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

Changelog:
2022-10-12: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "joystick_base.h"

#include <serial/serial.h>
#include <thread>
#include <mutex>

class JoystickSerial : public JoystickBase
{
public:
	JoystickSerial() = delete;
	JoystickSerial(std::shared_ptr<ros::NodeHandle>& NodeHandle,
		const std::string& Name, const std::string& Topic, int DeviceId, uint8_t DataHead,
		const std::string& Port, unsigned int Baudrate = 9600);
	~JoystickSerial() override;

public:
	// override
	void fire() override;
	void publish() override;

protected:
	void sendCommand(uint8_t Id, uint8_t Len, uint8_t* Data);
	void fetchData(unsigned char* Data, size_t Length);
	void threadReadSerial();

protected:
	std::string serial_port_{ "" };
	unsigned int baudrate_{ 9600 };
	std::shared_ptr<serial::Serial> serial_inst_{ nullptr };
	std::mutex mtx_;
	int device_id_{ 0x01 };
	uint8_t pack_head_{ 0xff };
	std::size_t pack_length_{ 10 };
	std::thread th_read_;
	std::atomic_bool terminated_{ false };
	std::array<int16_t, 2> offsets_{ 0x800, 0x800 };
};
