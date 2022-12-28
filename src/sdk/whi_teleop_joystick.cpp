/******************************************************************
joystick interface under ROS 1

Features:
- abstract joystick interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_teleop_joystick/whi_teleop_joystick.h"
#include "whi_teleop_joystick/joystick_serial.h"

#include <functional>

namespace whi_motion_interface
{
    const char* Joystick::hardware[HARDWARE_SUM] = { "onboard", "i2c", "canbus", "serial", "usbcan" };

    Joystick::Joystick(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    void Joystick::init()
    {
        // common params
        std::string dataTopic;
        double maxLinear = 0.0;
        double maxAngular = 0.0;
        std::vector<int> ranges;
        bool dampingLinear = false;
        bool dampingAngular = false;
        JoystickBase::ButtonList buttons;
        node_handle_->param("/whi_joystick_hardware/data_topic", dataTopic, std::string("joystick_data"));
        node_handle_->param("/whi_joystick_hardware/max_linear", maxLinear, 0.3);
        node_handle_->param("/whi_joystick_hardware/max_angular", maxAngular, 1.0);
        node_handle_->getParam("/whi_joystick_hardware/ranges", ranges);
        node_handle_->param("/whi_joystick_hardware/damping_linear", dampingLinear, false);
        node_handle_->param("/whi_joystick_hardware/damping_angular", dampingAngular, false);
        node_handle_->getParam("/whi_joystick_hardware/buttons", buttons);

        std::string hardwareStr;
        node_handle_->param("/whi_joystick_hardware/hardware", hardwareStr, std::string(hardware[ON_BOARD]));
        if (hardwareStr == hardware[SERIAL])
        {
            std::string name;
            std::string port;
            int baudrate = 9600;
            int deviceId = 0x01;
            int dataHead = 0xff;
            node_handle_->param("/whi_joystick_hardware/serial/name", name, std::string("unspecified"));
            node_handle_->param("/whi_joystick_hardware/serial/port", port, std::string("/dev/ttyUSB0"));
            node_handle_->param("/whi_joystick_hardware/serial/baudrate", baudrate, 9600);
            node_handle_->param("/whi_joystick_hardware/serial/device_id", deviceId, -1);
            node_handle_->param("/whi_joystick_hardware/serial/data_head", dataHead, 0xff);

            joystick_inst_ = std::make_unique<JoystickSerial>(node_handle_, name, dataTopic, deviceId, (uint8_t)dataHead, port, baudrate);
            joystick_inst_->setRanges(std::array<uint16_t, 2>({ uint16_t(ranges[0]), uint16_t(ranges[1]) }));
            joystick_inst_->setKinematicsLimits(maxLinear, maxAngular);
            joystick_inst_->setButtons(buttons);
            joystick_inst_->setDamping(dampingLinear, dampingAngular);
            ((JoystickSerial*)joystick_inst_.get())->fire();
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("failed to init driver of %s", hardwareStr.c_str());
        }

        // spinner
        node_handle_->param("/whi_joystick_hardware/loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&Joystick::update, this, std::placeholders::_1)));
    }

    void Joystick::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        joystick_inst_->publish();
    }
}
