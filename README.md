# ROS TurtleBot3 GUI Controller (Windows)

This project presents a ROS 2 (Humble) simulation of a TurtleBot3 Burger robot
controlled via a custom graphical user interface.

## Features
- TurtleBot3 Burger simulation in Gazebo
- Custom Python GUI acting as a 2D joystick
- Click-and-drag mouse control
- Continuous velocity publishing to `/cmd_vel`
- Single launch file to start the whole system

## How to run

```bash
#running docker

docker start ros2_lab
docker exec -it ros2_lab bash

#setting evething up

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

#lounching gazebo + robot + controller GUI

ros2 launch click_controller demo.launch.py

```

