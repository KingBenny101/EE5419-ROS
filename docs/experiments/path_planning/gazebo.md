# Gazebo Simulation for TurtleBot3

## Setting Up the Environment

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

## Launching Simulation World

- Launch the Gazebo simulation with the TurtleBot3 Burger model

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## Operate using Teleop

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

## Reseting the Simulation World

- If you want to reset the simulation world, you can use the following command in a new terminal:

```bash
rosservice call /gazebo/reset_world
```

- You can also reset from the Gazebo GUI by selecting `Edit` -> `Reset World`.
