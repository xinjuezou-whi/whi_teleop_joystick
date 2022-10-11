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
        // drivers
        //std::string frameId;
        //std::string dataTopic;
        //int chargeThresh;
        //node_handle_->param("/whi_battery_monitor/frame_id", frameId, std::string("base_link"));
        //node_handle_->param("/whi_battery_monitor/data_topic", dataTopic, std::string("battery_data"));
        //node_handle_->param("/whi_battery_monitor/charge_thresh", chargeThresh, 20);

        //std::string hardwareStr;
        //node_handle_->param("/whi_battery_monitor/hardware", hardwareStr, std::string(hardware[ON_BOARD]));
        //if (hardwareStr == hardware[CAN_BUS])
        //{
        //    std::string busAddr;
        //    int deviceAddr = 0;
        //    node_handle_->param("/whi_battery_monitor/canbus/bus_addr", busAddr, std::string("can0"));
        //    node_handle_->param("/whi_battery_monitor/canbus/device_addr", deviceAddr, 0);

        //    battery_inst_ = std::make_unique<BatteryCanbus>(node_handle_, busAddr, deviceAddr);
        //    battery_inst_->setChargeThresh((uint16_t)(chargeThresh));
        //}
        //else
        //{
        //    ROS_FATAL_STREAM_NAMED("failed to init driver of %s", hardwareStr.c_str());
        //}
        //battery_inst_->setPublishParams(frameId, dataTopic);

        // spinner
        node_handle_->param("/whi_battery_monitor/loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&Joystick::update, this, std::placeholders::_1)));
    }

    void Joystick::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        //battery_inst_->request();
    }
}
