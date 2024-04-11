#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopTurtle:

    def __init__(self):
        rospy.init_node('test_teleop_joy_turtle')
        self.publisher_ = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.subscriber_ = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        msg = Twist()
        msg.linear.x = 2 * data.axes[1]
        msg.angular.z = 2 * data.axes[0]

        self.publisher_.publish(msg)
        rospy.loginfo('Linear: "%f"' % msg.linear.x)
        rospy.loginfo('Angular: "%f"' % msg.angular.z)

def main():
    try:
        joystick = TeleopTurtle()
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

