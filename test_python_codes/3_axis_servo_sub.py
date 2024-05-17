#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamixel_sdk import *  # Dynamixel SDK library
from joy_custom_msg.msg import JoystickValues  # Custom message for joystick values

# Define Dynamixel control parameters
MY_DXL = 'X_SERIES'  # Use X-Series Dynamixel
ADDR_GOAL_POSITION         = 116  # Control table address for goal position
ADDR_PRESENT_POSITION      = 132  # Control table address for present position
DXL_MINIMUM_POSITION_VALUE = 0  # Minimum position limit
DXL_MAXIMUM_POSITION_VALUE = 4095  # Maximum position limit
BAUDRATE                   = 115200  # Baudrate for communication
ADDR_TORQUE_ENABLE         = 64
TORQUE_ENABLE              = 1     # Value for enabling the torque
TORQUE_DISABLE             = 0     # Value for disabling the torque
DEVICENAME = '/dev/ttyUSB0'  # Port device name
MIN_DEG = 180
MAX_DEG = 360

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)  # Protocol version 2.0

def initialize_dynamixel():
    # Open port
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.logerr("Failed to open the port")
        rospy.signal_shutdown("Failed to open the port")
        return False

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        rospy.signal_shutdown("Failed to change the baudrate")
        return False

    # Enable Dynamixel Torque
    for dxl_id in [11, 12, 13]:  # DXL_IDs for Dynamixel motors
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to enable Dynamixel torque for ID %d: %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
            return False

    rospy.loginfo("Dynamixel motors have been successfully connected")
    return True

def callback_joystick_values(data):
    # Extract values from JoystickValues message
    roll_value = data.roll
    pitch_value = data.pitch
    yaw_value = data.yaw
    
    # Convert roll, pitch, yaw from -180 to 180 to 0 to 4095 range
    def convert_to_motor_value(input_value):
        max_value = DXL_MAXIMUM_POSITION_VALUE
        converted_value = int(((input_value + MIN_DEG) * max_value) / MAX_DEG)
        
        # Check and limit value within the range
        converted_value = max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, converted_value))        
        return max(DXL_MINIMUM_POSITION_VALUE, min(max_value, converted_value))

    # Convert roll, pitch, yaw values
    convert_roll = convert_to_motor_value(roll_value)
    convert_pitch = convert_to_motor_value(pitch_value)
    convert_yaw = convert_to_motor_value(yaw_value)
    
    # Write goal position to Dynamixel with corresponding DXL_ID
    for dxl_id, goal_position in zip([11, 12, 13], [convert_roll, convert_pitch, convert_yaw]):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to write goal position for DXL_ID %d: %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logerr("Error in response for DXL_ID %d: %s" % (dxl_id, packetHandler.getRxPacketError(dxl_error)))

def dynamixel_control_node():
    rospy.init_node('dynamixel_control')

    # Initialize Dynamixel motors
    if not initialize_dynamixel():
        return

    # Subscribe to rostopic for joystick values
    rospy.Subscriber('/joystick_values', JoystickValues, callback_joystick_values)

    # Keep ROS node running
    rospy.spin()

    # Disable Dynamixel Torque
    for dxl_id in [11, 12, 13]:  # DXL_IDs for Dynamixel motors
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to disable Dynamixel torque for ID %d: %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))

    # Close port
    portHandler.closePort()
    rospy.loginfo("Dynamixel control node has been shut down")

if __name__ == '__main__':
    try:
        dynamixel_control_node()
    except rospy.ROSInterruptException:
        pass
