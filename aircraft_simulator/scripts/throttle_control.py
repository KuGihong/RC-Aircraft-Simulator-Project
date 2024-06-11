#!/usr/bin/env python3

'''''''''''''''''''''''''''''''''''''''''''''
$sudo pigpiod
$sudo killall pigpiod
pigpio의 데몬이 로우레벨언어로 하드웨어 동작과 연산을 나눠줌

set_servo_pulsewidth(user_gpio, pulsewidth)
Parameters

 user_gpio:= 0-31. -> 매칭채널
pulsewidth:= 0 (off),
             500 - 2500
'''''''''''''''''''''''''''''''''''''''''''''

import pigpio
import rospy
from joy_custom_msg.msg import JoystickValues  # Custom message for joystick values

rc_motor = 16

pi = pigpio.pi()
pi.set_servo_pulsewidth(rc_motor, 0)

input_min = 0
input_max = 200
output_min = 1130
output_max = 1180

def callback_joystick_values(data):
    vel_value = data.target_vel
    
    # Convert degree from range -180 to 180 to range 500 to 1500
    pulsewidth = map_range(vel_value)
    print(pulsewidth)
    pi.set_servo_pulsewidth(rc_motor, pulsewidth)

def throttle_control_node():
    rospy.init_node('throttle_control')   
    rospy.Subscriber('/joystick_values', JoystickValues, callback_joystick_values)
    
    # Keep ROS node running
    rospy.spin()
    rospy.loginfo("servo_control node has been shut down")
    
def map_range(value):
    # 디버깅: map_range 함수의 입력과 출력 값 출력
    mapped_value = ((value - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min
    return mapped_value

if __name__ == '__main__':
    try:
        throttle_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
         pi.set_servo_pulsewidth(rc_motor, 0)
         pi.stop()

