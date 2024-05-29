#!/usr/bin/env python3

'''''''''''''''''''''''''''''''''''''''''''''
$sudo pigpiod
$sudo killall pigpiod
pigpio의 데몬이 로우레벨언어로 하드웨어 동작과 연산을 나눠줌

set_servo_pulsewidth(user_gpio, pulsewidth)
Parameters

 user_gpio:= 0-31. -> 매칭채널
pulsewidth:= 0 (off),
             500 (0도) - 2500 (180도).
'''''''''''''''''''''''''''''''''''''''''''''

import pigpio
import rospy
from joy_custom_msg.msg import JoystickValues  # Custom message for joystick values

left_ailerron_servo = 18
right_ailerron_servo = 19
elevator_servo = 20
rudder_servo = 21

pi = pigpio.pi()
max_degree = 180
sum_degree = 360
roll_max_purse_wide = 1000 # 1000+535
roll_min_purse_wide = 535

def callback_joystick_values(data):
    roll_value = data.roll
    
    # Convert degree from range -180 to 180 to range 500 to 1500
    pulsewidth = ((roll_value + max_degree) * roll_max_purse_wide) / sum_degree + roll_min_purse_wide
    
    pi.set_servo_pulsewidth(left_ailerron_servo, pulsewidth)

def servo_control_node():
    rospy.init_node('servo_control')   
    rospy.Subscriber('/joystick_values', JoystickValues, callback_joystick_values)
    
    # Keep ROS node running
    rospy.spin()
    rospy.loginfo("servo_control node has been shut down")

if __name__ == '__main__':
    try:
        servo_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

