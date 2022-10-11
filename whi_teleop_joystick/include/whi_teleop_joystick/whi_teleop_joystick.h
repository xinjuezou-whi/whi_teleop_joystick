/******************************************************************
joystick interface under ROS 1

Features:
- abstract joystick interfaces
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
//#include "battery_base.h"

namespace whi_motion_interface
{
    class Joystick
    {
    public:
        enum Hardware { ON_BOARD = 0, I2C, CAN_BUS, SERIAL, USBCAN, HARDWARE_SUM };
        static const char* hardware[HARDWARE_SUM];

    public:
        Joystick(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~Joystick() = default;

    protected:
        void init();
        void update(const ros::TimerEvent& Event);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        //std::unique_ptr<BetteryBase> battery_inst_{ nullptr };
    };
}
