# whi_teleop_joystick
Node of processing signals of joystick and publish the cmd_vel message

Current support joystick series is SMC60:

![joystick_smc60](https://user-images.githubusercontent.com/72239958/195489669-013c6687-477b-46b8-96e7-410f3fea0465.jpg)


Such series has output options including CAN, RS-232/422/485, USB, by far only serial transmission is implemented. Other transmission implementation would be introduced following the requirements change of project

## Damping
For better controlling, this node add damping effector by sampling the linear and angular velocity with a cubic function:
![resistance](https://user-images.githubusercontent.com/72239958/195496660-91dfd4a4-c486-4bf1-a428-1f469e805df9.png)


## Config
yaml file is used to bearing the parameters including common ones and specified ones. Belowing is an example of config file:
```
whi_joystick_hardware:
  loop_hz: 50 #hz the frequency of publish
  data_topic: cmd_vel # topic of publish
  max_linear: 0.3 #m/s
  max_angular: 1.0 #rad/s
  ranges: [1951, 1951] # ranges of x and y
  buttons: {'reserved': 0x01, 'enable': 0x40} # buttons on joystick mount
  damping_linear: true
  damping_angular: true
  hardware: serial # transmission type
  serial:
    name: smc60_one_button
    port: /dev/ttyUSB0
    baudrate: 115200
    device_id: 0x01 # device ID of joystick mount
    data_head: 0xff # the head of data packet
```

The config file can be loaded from launch file and will be saved in rosparam server

## Setup
Go into your catkin workspace and initialize wstool if necessary (assuming ~/catkin_workspace as workspace path):
```
cd ~/catkin_workspace/src
git clone https://github.com/xinjuezou-whi/whi_teleop_joystick
```
Install missing packages with rosdep:
```
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```
Build the workspace:
```
cd ..
catkin build
source ~/catkin_workspace/devel/setup.bash
```

## Usage
Once build is succeed and the serial port settings is correctly configured, you can run its launch file to active the node:
```
cd ~/catkin_workspace
roslaunch whi_teleop_joystick whi_joystick.launch
```

Open another terminal to check the published message:
```
rostopic echo /cmd_vel
```

![joystick](https://user-images.githubusercontent.com/72239958/195493131-8a03da52-5290-4144-8bc0-0aa3335035fb.gif)


