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
            'left_ailerron_servo': {'max_pulse_width': 1000, 'min_pulse_width': 535},
            'right_ailerron_servo': {'max_pulse_width': 1000, 'min_pulse_width': 535},
            'elevator_servo': {'max_pulse_width': 1300, 'min_pulse_width': 900},
            'rudder_servo': {'max_pulse_width': 1400, 'min_pulse_width': 885}
        }
        self.max_degree = 180
        self.sum_degree = 360

    def convert_pulsewidth(self, servo_name, degree):
        params = self.servo_params[servo_name]
        max_pulse_width = params['max_pulse_width']
        min_pulse_width = params['min_pulse_width']
        return ((degree + self.max_degree) * max_pulse_width) / self.sum_degree + min_pulse_width

    def set_servo_pulsewidth(self, servo_name, degree):
        # Reverse the degree for right_ailerron_servo
        if servo_name == 'left_ailerron_servo' or 'elevator_servo' or 'rudder_servo':
            degree = -degree
        pulsewidth = self.convert_pulsewidth(servo_name, degree)
        self.pi.set_servo_pulsewidth(self.servos[servo_name], pulsewidth)

    def callback_joystick_values(self, data):
        print(f"Roll: {data.roll}, Pitch: {data.pitch}, Yaw: {data.yaw}, Throttle: data.throttle")
        
        self.set_servo_pulsewidth('left_ailerron_servo', data.roll)
        self.set_servo_pulsewidth('right_ailerron_servo', data.roll)
        self.set_servo_pulsewidth('elevator_servo', data.pitch)
        self.set_servo_pulsewidth('rudder_servo', data.yaw)

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

