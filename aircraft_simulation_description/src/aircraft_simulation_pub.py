#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from joy_custom_msg.msg import JoystickValues  # Custom message for joystick values
import math

def degree_to_radian(degree):
    return degree * (math.pi / 180.0)

def callback(data):
    # roll, pitch, yaw 값을 도(degree)에서 라디안(radian)으로 변환
    roll_radian = degree_to_radian(data.roll)
    pitch_radian = degree_to_radian(data.pitch)
    yaw_radian = degree_to_radian(data.yaw)
    
    # 변환된 라디안 값을 각각의 토픽에 publish
    pub_roll.publish(-roll_radian)
    pub_pitch.publish(-pitch_radian)
    pub_yaw.publish(-yaw_radian)

    rospy.loginfo("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(roll_radian, pitch_radian, yaw_radian))

if __name__ == '__main__':
    rospy.init_node('joystick_to_radian_converter', anonymous=True)
    
    # JoystickValues 메시지를 구독(subscribe)
    rospy.Subscriber('/joystick_values', JoystickValues, callback)
    
    # 각 관절의 위치 제어 토픽에 publish
    pub_roll = rospy.Publisher('/aircraft/roll_joint_position_controller/command', Float64, queue_size=10)
    pub_pitch = rospy.Publisher('/aircraft/pitch_joint_position_controller/command', Float64, queue_size=10)
    pub_yaw = rospy.Publisher('/aircraft/yaw_joint_position_controller/command', Float64, queue_size=10)
    
    rospy.spin()
