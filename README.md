teleop_acker_joy
================

# Overview
The purpose of this package is to provide a generic facility for tele-operating Ackermann-based ROS2 robots with a standard joystick.
It converts joy messages to Ackermann velocity commands.

This node provides no rate limiting or autorepeat functionality. It is expected that you take advantage of the features built into [joy](https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/#foxy) for this.

## Executables
The package comes with the `teleop_node` that republishes `sensor_msgs/msg/Joy` messages as scaled `ackermann_msgs/msg/AckermannDriveStamped` messages.

## Subscribed Topics
- `joy (sensor_msgs/msg/Joy)`
  - Joystick messages to be translated to velocity commands.

## Published Topics
- Base path: `cmd/`
- `cmd/vel (ackermann_msgs/msg/AckermannDriveStamped)`
  - Command velocity messages arising from Joystick commands.
- `cmd/headlight (std_msgs/msg/Bool)`
- `cmd/turnsignal_left (std_msgs/msg/Bool)`
- `cmd/turnsignal_right (std_msgs/msg/Bool)`
- `cmd/flash (std_msgs/msg/Bool)`

## Parameters (currently in the flow!)
- `require_enable_button (bool, default: true)`
  - Whether to require the enable button for enabling movement.

- `enable_button (int, default: 0)`
  - Joystick button to enable regular-speed movement.

- `enable_turbo_button (int, default: -1)`
  - Joystick button to enable high-speed movement (disabled when -1).

- `lights.<thing>`
  - Define lights to map
  - `lights.headlight            (int, default -1)`
  - `lights.turn_left            (int, default -1)`
  - `lights.turn_right           (int, default -1)`
  - `lights.flash                (int, default -1)`

- `axis.<thing>`
  - Define the joystick axis to use
  - `axis.linear                  (int, default  5)`
  - `axis.steering_angle          (int, default  6)`
  - `axis.steering_angle_fine     (int, default -1)`
  - `axis.steering_angle_velocity (int, default -1)`

- `scale.<thing>`
  - Scale to apply to joystick axis for regular-speed movement.
  - `scale.linear                  (double, default 0.5)`
  - `scale.steering_angle          (double, default 1.4)`
  - `scale.steering_angle_fine     (double, default 0.0)`
  - `scale.steering_angle_velocity (double, default 0.0)`

- `scale_turbo.<thing>`
  - Scale to apply to joystick axis for _turbo_-speed movement.
  - `scale_turbo.linear                  (double, default 1.0)`
  - `scale_turbo.steering_angle          (double, default 0.8)`
  - `scale_turbo.steering_angle_fine     (double, default 0.0)`
  - `scale_turbo.steering_angle_velocity (double, default 0.0)`

- `offset.<thing>`
  - Offset to apply to joystick axis.
  - `offset.linear                  (double, default 0.0)`
  - `offset.steering_angle          (double, default 0.0)`
  - `offset.steering_angle_fine     (double, default 0.0)`
  - `offset.steering_angle_velocity (double, default 0.4)`

# Usage

## Install
For most users building from source will not be required, execute `apt-get install ros-<rosdistro>-teleop-twist-joy` to install.

## Run
A launch file has been provided which has three arguments which can be changed in the terminal or via your own launch file.
To configure the node to match your joystick a config file can be used.
There are several common ones provided in this package (atk3, ps3-holonomic, ps3, xbox, xd3), located here: https://github.com/ros2/teleop_acker_joy/tree/eloquent/config.

PS3 is default, to run for another config (e.g. xbox) use this:
````
ros2 launch teleop_acker_joy teleop-launch.py joy_config:='xbox'
````

__Note:__ this launch file also launches the `joy` node so do not run it separately.


## Arguments
- `joy_config (string, default: 'ps3')`
  - Config file to use
- `joy_dev (string, default: 'dev/input/js0')`
  - Joystick device to use
- `config_filepath (string, default: '/opt/ros/<rosdistro>/share/teleop_acker_joy/config/' + LaunchConfig('joy_config') + '.config.yaml')`
  - Path to config files
