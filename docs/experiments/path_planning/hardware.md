# Hardware for TurtleBot3

## Objectives

- To control multiple TurtleBot3 robots through the ROS network.

## Experimental Setup

- The master node (computer) and the control nodes (other computers) are connected to the same network.
- The master node is running the ROS master.
- The control nodes will be used to control the TurtleBot3 robots.
- All the TurtleBot3 robots are connected to the same network.
- Each TurtleBot3 robot has a unique IP address.
- Each TurtleBot3 robot is connected to a different control node.

## Master Node Setup

- Start the ROS master node by running the following command on the master node:

```bash
roscore
```

## Control Node Setup

- Set the following environment variables on each control node:

```bash
export ROS_MASTER_URI=http://<master_node_ip>:11311
export ROS_HOSTNAME=<control_node_ip>
export TURTLEBOT3_MODEL=burger
```

- Replace `<master_node_ip>` with the IP address of the master node and `<control_node_ip>` with the IP address of the control node.
- You can find the IP address of the nodes by running the following command on the nodes:

```bash
ifconfig
```

## TurtleBot3 Setup

- Power on the TurtleBot3 robot by connecting the battery.
- The robot will automatically boot up and connect to the network.
- Open a terminal on either the master node or the control node and ssh into the TurtleBot3 robot:

```bash
ssh ubuntu@<robot_ip>
```

- Replace `<robot_ip>` with the IP address of the TurtleBot3 robot.
- You will be prompted to enter the password. The default password is `turtlebot`.

- Set the following environment variables on the TurtleBot3 robot:

```bash
export ROS_MASTER_URI=http://<master_node_ip>:11311
export ROS_HOSTNAME=<robot_ip>
export TURTLEBOT3_MODEL=burger
```

- Replace `<master_node_ip>` with the IP address of the master node and `<robot_ip>` with the IP address of the TurtleBot3 robot.

Generally these configurations are set in the `~/.bashrc` file, so they are automatically set when the terminal is opened.

- Bring up TurtleBot3 robot, this callibrates the robot and starts the ROS nodes:

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

## Controling the TurtleBot3 Robot

- Each TurtleBot3 robot is namespaced with unique identifiers.
- Here they are named  `tb3_1`,`tb3_2`,`tb3_3`,`tb3_4`.
- For example, to control the first robot, you have to publish to the `/tb3_1/cmd_vel` topic.
- To get Odometry data from the second robot, you have to subscribe to the `/tb3_2/odom` topic.
- To get a list of all the topics, you can run the following command on any of the nodes:

```bash
rostopic list
```
