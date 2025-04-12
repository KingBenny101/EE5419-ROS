# Path Planning with Turtlebot3 in Gazebo

## Objectives

- Understand the basic hardware and software architecture of TurtleBot3.
- Learn how to set up and simulate TurtleBot3 in Gazebo for path planning experiments.
- Practice commanding the TurtleBot3 to follow a trajectory without using the built-in ROS Navigation Stack.
- Transition from simulation to a physical TurtleBot3 robot for real-world testing and comparison.

## Theory

### Introduction to TurtleBot3

TurtleBot3 is a small, affordable, and fully open-source ROS-based mobile robot. It typically includes:

- **Single-board Computer:** A Raspberry Pi running Ubuntu and ROS.
- **OpenCR Board:** Handles direct motor control, sensor reading, and low-level feedback loops.
- **2WD Differential-Drive Base:** Uses two DC motors for locomotion.
- **LIDAR Sensor:** Provides distance measurement and environment mapping.

By publishing velocity commands on the `/cmd_vel` topic, you can manually steer the robot in Gazebo (a 3D physics simulator) and eventually apply the same concept on physical hardware. This approach provides deeper insight into differential-drive kinematics, odometry, and the differences between simulated and real-world performance.

### Key Features of TurtleBot3

- **Modular Architecture:** The same code typically works both in simulation (Gazebo) and on real hardware.
- **Differential Drive:** Simplifies forward and turn commands; however, it can be prone to wheel slip.
- **ROS Topics:**
  - `/cmd_vel`: Publishes `geometry_msgs/Twist` messages to set linear and angular velocities.
  - `/odom`: Provides odometry updates for position and heading estimation.
- **Open-Source Tools:** The entire TurtleBot3 software stack (URDF files, Gazebo worlds, driver nodes) is freely available.

**Note** In advanced projects, you might use the standard Navigation Stack (such as `move_base` and costmaps). Here, however, we bypass it to understand how raw velocity commands influence a differential-drive robot. For this exercise, **ROS Noetic** is preferred due to its extended support for TurtleBot3 in both simulations and hardware setups.

### References

- [TurtleBot3 E-Manual (Robotis)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [TurtleBot3 ROS Wiki](https://wiki.ros.org/turtlebot3)
- [ROS Tutorial: Publisher/Subscriber](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

## Installation and Setup

- Make sure to follow the instructions corresponding to your ROS version, this document is based on **ROS Noetic**.

- Install Turtlebot3 Packages required for the Hardware and Simulation Setups by following the steps in the [TurtleBot3 Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

- Set up the simulation environment by following the steps in the [TurtleBot3 Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) section of the E-Manual.

### Setting Up the Environment

- Create a catkin workspace and clone the required repository

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

- Source the workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

### Launching Simulation World

- Launch the Gazebo simulation with the TurtleBot3 Burger model

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Operate using Teleop

- Open a new terminal and source the workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

- Set the model environment variable

```bash
export TURTLEBOT3_MODEL=burger
```

- Launch the teleop node

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

- You can follow the instructions in the terminal to control the robot using the keyboard.

### Reseting the Simulation World

- If you want to reset the simulation world, you can use the following command in a new terminal:

```bash
rosservice call /gazebo/reset_world
```

- You can also reset from the Gazebo GUI by selecting `Edit` -> `Reset World`.
