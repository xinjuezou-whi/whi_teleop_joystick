/******************************************************************
node to get the signals from joystick and send out cmd_vel

Features:
- communication resource setup
- publish message on cmd_vel
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-11: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <iostream>
#include <functional>
#include <signal.h>

#include "whi_teleop_joystick/whi_teleop_joystick.h"

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI teleop of joystick VERSION 00.01" << std::endl;
	std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
	ros::init(argc, argv, "whi_teleop_joystick");
	auto nodeHandle = std::make_shared<ros::NodeHandle>();

	/// node logic
	auto joystick = std::make_unique<whi_motion_interface::Joystick>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
	// NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		joystick = nullptr;

		// all the default sigint handler does is call shutdown()
		ros::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(0);
	spinner.spin();
#endif

	std::cout << "whi_teleop_joystick exited" << std::endl;

	return 0;
}
