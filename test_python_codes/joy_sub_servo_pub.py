#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

# Callback function for subscribing to position_value topic
def position_value_callback(msg):
    global goal_position
    # Set goal_position to the received position_value
    goal_position = msg.data

def publish_goal_position():
    rospy.init_node('goal_position_publisher')
    pub = rospy.Publisher('goal_position', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Subscribe to position_value topic
    rospy.Subscriber('joystick_value', Int32, position_value_callback)

    while not rospy.is_shutdown():
        # Publish the goal_position
        pub.publish(goal_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize goal_position to a default value
        goal_position = 0
        publish_goal_position()
    except rospy.ROSInterruptException:
        pass

