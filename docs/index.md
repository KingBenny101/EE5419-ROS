# EE5419 Advanced Control Laboratory – IIT Madras

This is a guide for experiments related to ROS in the [EE5419 Advanced Control Laboratory](https://www.ee.iitm.ac.in/courses/syllabus/EE5419) course at IIT Madras.

For the original documentation visit [original docs](https://docs.google.com/document/d/1pQMcXM4Eel9xM6bWdggSrqgIPPWc9-T8HNgE54NhdSk/edit?pli=1&tab=t.0).

## Theory

**Robot Operating System (ROS)** is an open-source framework widely used in both research and industry for robotic system development. While it's called an "operating system," ROS is actually a middleware that runs on top of Linux-based systems such as Ubuntu.

It simplifies communication between different components of a robotic system—like sensors, actuators, and controllers.

## Key Features

- **Node-based Communication**: Components (called _nodes_) communicate using topics, services, or actions.
- **Standardized Libraries**: Offers pre-built libraries for tasks such as motion planning, navigation, and perception.
- **Simulators**: Integrates with simulators like Gazebo for testing robot behavior in virtual environments.
- **Modularity**: Facilitates easy integration of various hardware and software components.
- **Community Support**: Backed by a large community with rich documentation and numerous open-source packages.

## ROS Versions

- **ROS 1**: The original version, still widely used.
- **ROS 2**: Offers better real-time support, enhanced security, and improved multi-robot communication. Designed to meet the demands of modern robotic systems.

## Tested Configuration

- **Operating System**: Ubuntu 20.04.6 LTS
- **ROS Version**: ROS Noetic Ninjemys
- **Python Version**: Python 3.8.10

You are advised to use the same configuration for this course but you are free to use any other configuration as long as you can run the code provided in this repository.
