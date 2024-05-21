#!/usr/bin/env python3
import rospy
from joy_custom_msg.msg import JoystickValues  # 새로운 메시지 형식을 import

from sensor_msgs.msg import Joy

class JoystickConverter:

    def __init__(self):
        rospy.init_node('joystick_converter')
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.publisher = rospy.Publisher('/joystick_values', JoystickValues, queue_size=10)
        self.subscriber = rospy.Subscriber('/stick/joy', Joy, self.joy1_callback)
        self.subscriber = rospy.Subscriber('/pedal/joy', Joy, self.joy2_callback)
        
        self.input_min = -1
        self.input_max = 1
        self.output_min = -180
        self.output_max = 180
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.joy_custom_pub)

    def joy1_callback(self, data):
        self.roll = self.map_range(data.axes[0])  # 0번 축
        self.pitch = self.map_range(data.axes[1])  # 1번 축

    def joy2_callback(self, data):
        self.yaw = self.map_range(data.axes[2])  # 2번 축
		
    def map_range(self, value):
        # 디버깅: map_range 함수의 입력과 출력 값 출력
        mapped_value = ((value - self.input_min) / (self.input_max - self.input_min)) * (self.output_max - self.output_min) + self.output_min
        return mapped_value
        
    def joy_custom_pub(self, event):
	    # 조이스틱 값을 새로운 메시지 형식에 저장
        joystick_msg = JoystickValues()
        joystick_msg.roll = self.roll  # 0번 축
        joystick_msg.pitch = self.pitch  # 1번 축
        joystick_msg.yaw = self.yaw

        # 변환된 값을 publish
        self.publisher.publish(joystick_msg)
        rospy.loginfo('Sidestick Values - roll: %f, pitch: %f, yaw: %f' % (joystick_msg.roll, joystick_msg.pitch, joystick_msg.yaw))
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
