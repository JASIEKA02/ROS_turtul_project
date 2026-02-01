# ROS 2 TurtleBot Control Project

## Overview
This project presents a **Docker-based ROS 2 Humble system** for controlling a **TurtleBot3** robot in simulation.
The robot motion is controlled using **user interaction published on a ROS topic**, following a simple and transparent control logic.

The entire system can be launched using **a single ROS 2 launch file**, which starts:
- the TurtleBot3 Gazebo simulation (headless),
- a custom ROS 2 control node.

The project was developed as part of an academic course on robotics and ROS 2 systems.

---

## Project Goals
- demonstrate understanding of ROS 2 node architecture,
- implement a custom control node in Python,
- use Docker to ensure reproducibility of the environment,
- integrate simulation and control using a launch file,
- provide a clear and minimal user interaction mechanism.

---

## System Architecture

+----------------------+

| User Input          |

| (/clicked_point)    |

+----------+-----------+

|

v

+------------------------------+

| Click Control Node   |

| (turtlebot_control)  |

|                      |

| Logic:               |

| y > 0 -> forward     |

| y < 0 -> backward    |

+--------------+---------------+

|

v

+------------------------------+

| /cmd_vel (Twist)     |

+--------------+---------------+

|

v

+------------------------------+

| TurtleBot3 Simulation |

| (Gazebo, headless)    |

+------------------------------+


---


### Control Logic

The control node subscribes to the topic:
/clicked_point (geometry_msgs/PointStamped)


Decision rule:
- if `point.y > 0` → robot moves **forward**,
- if `point.y < 0` → robot moves **backward**.

Velocity commands are published to:
/cmd_vel (geometry_msgs/Twist)
This simple logic allows easy testing and clear demonstration of ROS topic-based control.

---

### Project Structure


.

├── docker/

│ └── Dockerfile

├── src/

│ └── turtlebot_control/

│ ├── launch/

│ │ └── system.launch.py

│ ├── turtlebot_control/

│ │ └── control_node.py

│ ├── package.xml

│ ├── setup.py

│ └── setup.cfg

├── README.md

└── .gitignore



---

### Docker Environment

The project uses **Docker** to ensure:
- consistent ROS 2 version (Humble),
- reproducible dependencies,
- isolation from the host system.

---

### How to Run the Project

### Build Docker image
```bash
docker build -t turtlebot_ros:humble -f docker/Dockerfile 
```

### Run container
```bash
docker run -it \
  --net=host \
  -v $(pwd)/src:/ros2_ws/src \
  turtlebot_ros:humble
```

### Build workspace (inside container)
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

### Launch the system
```bash
ros2 launch turtlebot_control system.launch.py
```

This command starts:
TurtleBot3 simulation (Gazebo, without GUI),
the custom click-based control node.

---
### Testing the Control Logic

User input can be simulated from another terminal:
The robot to move forward.
```bash
ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  frame_id: 'map'
point:
  x: 0.0
  y: 1.0
  z: 0.0"
```


The robot to move backward.
```bash
ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  frame_id: 'map'
point:
  x: 0.0
  y: -1.0
  z: 0.0"
```



