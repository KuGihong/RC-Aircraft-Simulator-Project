#!/usr/bin/env python3
import rospy
from joy_custom_msg.msg import JoystickValues  # 커스텀 메시지 import

from sensor_msgs.msg import Joy

class JoystickConverter:

    def __init__(self):
        rospy.init_node('joystick_converter')
        self.publisher = rospy.Publisher('/joystick_values', JoystickValues, queue_size=10)
        self.subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        # 조이스틱 값을 새로운 메시지 형식에 저장
        joystick_msg = JoystickValues()
        joystick_msg.roll = int(((data.axes[0] + 1) / 2) * 4095)  # 0번 축
        joystick_msg.pitch = int(((data.axes[1] + 1) / 2) * 4095)  # 1번 축
        joystick_msg.yaw = int(((data.axes[3] + 1) / 2) * 4095)  # 1번 축

        # 변환된 값을 publish
        self.publisher.publish(joystick_msg)
        rospy.loginfo('Joystick Values - roll: %d, pitch: %d, yaw: %d' % (joystick_msg.roll, joystick_msg.pitch, joystick_msg.yaw))

def main():
    try:
        joystick_converter = JoystickConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interrupt")
    finally:
        rospy.loginfo("Shutting down...")
        rospy.sleep(1)

if __name__ == '__main__':
    main()
