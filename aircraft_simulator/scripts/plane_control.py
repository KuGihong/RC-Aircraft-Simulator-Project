#!/usr/bin/env python3
'''''''''''''''''''''''''''''''''''''''''''''
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

class ServoController:
    def __init__(self):
        self.pi = pigpio.pi()
        self.servos = {
            'left_ailerron_servo': 23,
            'right_ailerron_servo': 24,
            'elevator_servo': 25,
            'rudder_servo': 18
        }
        self.servo_params = {
            'left_ailerron_servo': {'max_pulse_width': 1870, 'min_pulse_width': 1200},
            'right_ailerron_servo': {'max_pulse_width': 1780, 'min_pulse_width': 1200},
            'elevator_servo': {'max_pulse_width': 1780, 'min_pulse_width': 1200},
            'rudder_servo': {'max_pulse_width': 1780, 'min_pulse_width': 1200}
        }
        self.min_input = -30
        self.max_input = 30

    def convert_pulsewidth(self, servo_name, degree):
        params = self.servo_params[servo_name]
        max_pulse_width = params['max_pulse_width']
        min_pulse_width = params['min_pulse_width']   
        mapped_value = min_pulse_width + (float(degree - self.min_input) / (self.max_input - self.min_input) * (max_pulse_width - min_pulse_width))
     
        return mapped_value
        
    def set_servo_pulsewidth(self, servo_name, degree):
        # Reverse the degree for right_ailerron_servo
        #if servo_name in['elevator_servo', 'rudder_servo']:
        #    degree = -degree
        pulsewidth = self.convert_pulsewidth(servo_name, -degree)
        self.pi.set_servo_pulsewidth(self.servos[servo_name], pulsewidth)

    def callback_joystick_values(self, data):
        print(f"Roll: {data.target_roll}, Pitch: {data.target_pitch}, Yaw: {data.target_yaw}, Throttle: {data.target_vel}")
        
        self.set_servo_pulsewidth('left_ailerron_servo', data.target_roll)
        self.set_servo_pulsewidth('right_ailerron_servo', data.target_roll)
        self.set_servo_pulsewidth('elevator_servo', data.target_pitch)
        self.set_servo_pulsewidth('rudder_servo', data.target_yaw)

    def servo_control_node(self):
        rospy.init_node('servo_control')
        rospy.Subscriber('/joystick_values', JoystickValues, self.callback_joystick_values)
        
        # Keep ROS node running
        rospy.spin()
        rospy.loginfo("servo_control node has been shut down")

    def cleanup(self):
        for servo_name in self.servos.values():
            self.pi.set_servo_pulsewidth(servo_name, 0)
        self.pi.stop()

if __name__ == '__main__':
    controller = ServoController()
    try:
        controller.servo_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()

