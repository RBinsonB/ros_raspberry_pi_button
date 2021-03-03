# raspberry_pi_button

## Overview
A ROS package to interface a button to a ROS topics using a Raspberry Pi GPIO. Provides an easy way to create button boxes and integrate them within ROS.

<a href="url"><img src="/documentation/pictures/button_box.jpg" align="center" height="300" width="400"></a>
<a href="url"><img src="/documentation/pictures/button_box_pushed.jpg" align="center" height="300" width="400"></a>
*Example of button box using a Raspberry Pi 3B+ and homemade case*

### License
MIT

## Installation
Package has to be launched from a Raspberry Pi. Has been tested on Raspberry Pi 3B+.

### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org)

- [RPi.GPIO](https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/)
	`sudo apt install rpi.gpio`

### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/RBinsonB/ros_raspberry_pi_button.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

### Setting up hardware
Connect any number of buttons to GPIO pins of the Raspberry Pi. Each button should supply 5V (from the Raspberry Pi 5V pins or other) to the input pin. The code provided here will read the pin value and uses Raspberry Pi internal pull-down resistors.

<a href="url"><img src="/documentation/pictures/button_box_internal.jpg" align="center" height="525" width="700"></a>
*Button wiring example for two buttons and a Raspberry Pi 3B+*

### Setting up the Raspberry Pi ROS environment
It is assumed that the ROS master is running on another computer on the same network. The Raspberry Pi ROS environment needs to be set accordingly.

* **From the Raspberry Pi**
  Set the ROS_MASTER_URI to the ROS master computer IP and port (replacing <ros-master-ip> by the proper ROS master computer IP): `export ROS_MASTER_URI=http://<ros-master-ip>:11311`

  Set the ROS_IP to the Raspberry Pi IP (replacing <rpi-ip> by the proper Raspberry Pi IP): `export ROS_IP=<rpi-ip>`

* **From the ROS master computer**
  Set the ROS_IP to the ROS master computer IP (replacing <ros-master-ip> by the proper ROS master computer IP): `export ROS_IP=<ros-master-ip>`

## Usage
Configuration is done with ROS parameters. Each button need the private parameters `/input_pin` and `/topic_id` grouped under the button name. For example to setup a button named `button_1`, reading value from a button connected to GPIO 4 and publishing it on the topic `button_state`, the parameters should be:
```
~/button_1/input_pin: 4
~/button_1/topic_id: button_state
```
YAML config files help setting up multiple buttons. An example config files for a box of two buttons can be found at [/config/button_box_example.yaml](/config/button_box_example.yaml)

To launch, simply run the node `button_node.py` (with the previously stated parameters as private parameters) while a ROS master core is running. For each button the state will be published as a boolean to the specified ROS topic.


### Example
An example launch file for a box of two buttons (connected on GPIO 4 and 27) can be found at [/launch/button_box_example.launch](/launch/button_box_example.launch) and run using the following command:
```
roslaunch raspberry_pi_button button_box_example.launch
```
Checking that button state are being published can be done by running:
```
rostopic list
```
The following topics should appear in the list.
```
/button_1
/button_2
```
Checking each button state can be done by running the following:
```
rostopic echo /button_1
```
and
```
rostopic echo /button_2
```


## Config files
- **button_box_example.yaml** Example of configuration file for a box of two buttons connected to GPIO 4 and 27.

## Launch files
- **button_box_example.launch**: Example of implementation of a box of two buttons. Run the button node with associated configuration to track the state of two buttons and publish the states to ROS topics.

## Nodes
### button_node.py
Track every configured button state and publish it over ROS as a Bool message. Message is latched and only published when button is changing state.

#### Published Topics
- `topic_id` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html), latched)

    Button state (`True` when circuit close, `False` when circuit open). Topic name is to be configured by the parameter `topic_id` and is subject to change. Each configured button gets its own state publisher topic.

#### Parameters
- `~/button_name/input_pin` (int)

    Button input pin using BCM numbering. `button_name` can be configured to any name. Parameter needed for each button.    

- `~/button_name/topic_id` (string)

    Button state topic. `button_name` can be configured to any name. Parameter needed for each button.    




