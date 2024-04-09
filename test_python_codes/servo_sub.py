#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamixel_sdk import * # Dynamixel SDK library
from std_msgs.msg import Int32

# Define Dynamixel control parameters
MY_DXL                     = 'X_SERIES'  # Use X-Series Dynamixel
DXL_ID                     = 11  # Default ID of Dynamixel
ADDR_GOAL_POSITION         = 116  # Control table address for goal position
ADDR_PRESENT_POSITION      = 132  # Control table address for present position
DXL_MINIMUM_POSITION_VALUE = 0  # Minimum position limit
DXL_MAXIMUM_POSITION_VALUE = 4095  # Maximum position limit
BAUDRATE                   = 115200  # Baudrate for communication
ADDR_TORQUE_ENABLE         = 64
TORQUE_ENABLE              = 1     # Value for enabling the torque
TORQUE_DISABLE             = 0     # Value for disabling the torque
DEVICENAME = '/dev/ttyUSB0'  # Port device name

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)  # Protocol version 2.0

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    #rospy.signal_shutdown("Failed to open the port")

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    #rospy.signal_shutdown("Failed to change the baudrate")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    print("Failed to enable Dynamixel torque")
elif dxl_error != 0:
    rospy.logerr("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Failed to enable Dynamixel torque")
else:
    print("Dynamixel has been successfully connected")

# Define callback function for rostopic subscriber
def callback_goal_position(data):
    goal_position = data.data
    # Check goal position limits
    if goal_position < DXL_MINIMUM_POSITION_VALUE:
        goal_position = DXL_MINIMUM_POSITION_VALUE
    elif goal_position > DXL_MAXIMUM_POSITION_VALUE:
        goal_position = DXL_MAXIMUM_POSITION_VALUE
    # Write goal position to Dynamixel
    if MY_DXL == 'XL320':
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.logerr("%s" % packetHandler.getRxPacketError(dxl_error))

# Initialize ROS node
rospy.init_node('dynamixel_control')

# Subscribe to rostopic for goal position
rospy.Subscriber('goal_position', Int32, callback_goal_position)

# Keep ROS node running
rospy.spin()

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    rospy.logerr("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

