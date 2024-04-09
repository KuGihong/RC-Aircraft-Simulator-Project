# RC-Aircraft-Simulator-Project

Applied Smart Mechanical Systems Engineering Graduation Topic

![image](https://github.com/KuGihong/RC-Aircraft-Simulator-Project/assets/113013130/0820162d-3dfd-40b9-a545-6a7834865664)

## Settings for using the joystick

### Install jstest-gtk

jstest-gtk is a utility for testing and configuring game controllers or joysticks on Linux systems.

    sudo apt update
    sudo apt install jstest-gtk

### Install ros-joy package

ros-joy is one of the Robot Operating System (ROS) packages that allows you to use joystick input in ROS systems.

    sudo apt install ros-noetic-joy

## DynamixelSDK with Python Examples

This repository contains modified Python example codes based on the DynamixelSDK from [ROBOTIS-GIT/DynamixelSDK.git](https://github.com/ROBOTIS-GIT/DynamixelSDK.git). The examples demonstrate how to control Dynamixel servos using Python.

### Installation

Clone the DynamixelSDK repository (including submodules) using Git:

    git clone --recursive https://github.com/ROBOTIS-GIT/DynamixelSDK.git

### Copying Python Examples to DynamixelSDK

Copy the Python examples from this repository into the `protocol2_0` directory of the DynamixelSDK.

#### Modified Python Examples

The `protocol2_0` folder in this repository contains modified Python example codes demonstrating various tasks using Dynamixel servos, including enhancements and customizations for specific applications.

#### License

The DynamixelSDK is licensed under the Apache License 2.0.
The modifications and custom code in the `protocol2_0` folder are authored by [KuGihong] and are also subject to the Apache License 2.0.

#### Usage

1. Ensure Python is installed on your system.
2. Navigate to the `protocol2_0` folder within the cloned DynamixelSDK repository.
3. Run the modified Python example scripts using the Python interpreter.
