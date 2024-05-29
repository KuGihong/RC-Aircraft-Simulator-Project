#!/usr/bin/env python3
import rospy
from joy_custom_msg.msg import JoystickValues  # 새로운 메시지 형식을 import
from sensor_msgs.msg import Joy
import time

class JoystickConverter:

    def __init__(self):
        rospy.init_node('joystick_converter')
        self.publisher = rospy.Publisher('/joystick_values', JoystickValues, queue_size=10)
        self.subscriber1 = rospy.Subscriber('/stick/joy', Joy, self.joy1_callback)
        self.subscriber2 = rospy.Subscriber('/pedal/joy', Joy, self.joy2_callback)
        
        # 현재 값과 목표 값 초기화
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # 각 축의 보간 상태 초기화
        self.start_roll = 0.0
        self.start_pitch = 0.0
        self.start_yaw = 0.0

        self.roll_start_time = time.time()
        self.pitch_start_time = time.time()
        self.yaw_start_time = time.time()

        self.roll_fraction = 0.0
        self.pitch_fraction = 0.0
        self.yaw_fraction = 0.0

        self.input_min = -1
        self.input_max = 1
        self.output_min = -90
        self.output_max = 90

        # 보간 시간과 주기 설정
        self.interpolation_time = 2.0  # 2초 동안 이동
        self.timer_period = 0.1  # 10ms 주기

        # 타이머 설정
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.update_values)

    def joy1_callback(self, data):
        self.target_roll = self.map_range(data.axes[0])  # 0번 축
        self.target_pitch = self.map_range(data.axes[1])  # 1번 축

        # 목표 값이 변경되었을 때 보간 초기화
        self.start_roll = self.current_roll
        self.start_pitch = self.current_pitch

        self.roll_start_time = time.time()
        self.pitch_start_time = time.time()

        self.roll_fraction = 0.0
        self.pitch_fraction = 0.0

    def joy2_callback(self, data):
        self.target_yaw = self.map_range(data.axes[2])  # 2번 축

        # 목표 값이 변경되었을 때 보간 초기화
        self.start_yaw = self.current_yaw

        self.yaw_start_time = time.time()
        self.yaw_fraction = 0.0

    def map_range(self, value):
        mapped_value = ((value - self.input_min) / (self.input_max - self.input_min)) * (self.output_max - self.output_min) + self.output_min
        return mapped_value

    def update_values(self, event):
        current_time = time.time()

        # roll 보간 업데이트
        roll_elapsed_time = current_time - self.roll_start_time
        if roll_elapsed_time < self.interpolation_time:
            self.roll_fraction = roll_elapsed_time / self.interpolation_time
        else:
            self.roll_fraction = 1.0

        self.current_roll = self.linear_interpolate(self.start_roll, self.target_roll, self.roll_fraction)

        # pitch 보간 업데이트
        pitch_elapsed_time = current_time - self.pitch_start_time
        if pitch_elapsed_time < self.interpolation_time:
            self.pitch_fraction = pitch_elapsed_time / self.interpolation_time
        else:
            self.pitch_fraction = 1.0

        self.current_pitch = self.linear_interpolate(self.start_pitch, self.target_pitch, self.pitch_fraction)

        # yaw 보간 업데이트
        yaw_elapsed_time = current_time - self.yaw_start_time
        if yaw_elapsed_time < self.interpolation_time:
            self.yaw_fraction = yaw_elapsed_time / self.interpolation_time
        else:
            self.yaw_fraction = 1.0

        self.current_yaw = self.linear_interpolate(self.start_yaw, self.target_yaw, self.yaw_fraction)

        joystick_msg = JoystickValues()
        joystick_msg.roll = self.current_roll
        joystick_msg.pitch = self.current_pitch
        joystick_msg.yaw = self.current_yaw

        self.publisher.publish(joystick_msg)
        rospy.loginfo('Sidestick Values - roll: %f, pitch: %f, yaw: %f' % (joystick_msg.roll, joystick_msg.pitch, joystick_msg.yaw))

    def linear_interpolate(self, start, end, fraction):
        return start + (end - start) * fraction

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
