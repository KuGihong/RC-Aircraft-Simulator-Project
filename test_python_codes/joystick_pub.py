#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

class JoystickConverter:

    def __init__(self):
        rospy.init_node('joystick_converter')
        self.publisher = rospy.Publisher('/joystick_value', Int32, queue_size=10)
        self.subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        # 조이스틱 값을 -32767 ~ 32767에서 0 ~ 4095로 변환
        scaled_value = int(((data.axes[0] + 1) / 2) * 4095)
        
        # 변환된 값을 publish
        self.publisher.publish(scaled_value)
        rospy.loginfo('Joystick Value: %d' % scaled_value)

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

