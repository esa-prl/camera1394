# camera1394 Driver 

## Overview

This packages allows to communicate with firewire cameras.

**Keywords:** camera, firewire, package

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Maximilian Ehrhardt<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Maximilian Ehrhardt, maximilian.ehrhardt@esa.int**

The camera1394 package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/camera1394.git
	cd ../
	colcon build --symlink-install

Adding `--symlink-install` to `colcon build` eliminates the need to recompile the package after changing the code.


## Usage

## Nodes

#### Published Topics

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_msgs/ChangeLocomotionMode]: https://github.com/esa-prl/rover_msgs/blob/master/srv/ChangeLocomotionMode.srv
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
