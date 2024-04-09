#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def publish_goal_position():
    rospy.init_node('goal_position_publisher')
    pub = rospy.Publisher('goal_position', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        goal_position = 0  # 예시로 중간 위치인 2048로 설정
        pub.publish(goal_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_position()
    except rospy.ROSInterruptException:
        pass

