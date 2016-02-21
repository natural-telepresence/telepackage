#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import mraa
import time


class motor_control(object):
    def __init__(self):
        #sub joystick x,y
        self.joy = rospy.Subscriber("/base/cmd_vel", Twist, self.callback)

        left_pin_num = rospy.get_param('left_motor_pin', 4) 
        left_motor_pin = mraa.Pwm(left_pin_num)
        left_motor_pin.period_us(20000)
        left_motor_pin.enable(True)

        right_pin_num = rospy.get_param('right_motor_pin', 3)
        right_motor_pin = mraa.Pwm(right_pin_num)
        right_motor_pin.period_us(20000)
        right_motor_pin.enable(True)

    def callback(self, data):
        fwd_speed = data.linear.x
        turn_speed = data.angular.z
        left_motor = fwd_speed + turn_speed
        right_motor = fwd_speed - turn_speed

        left_motor_pin.config_ms(20, 1.5 + 0.5 * left_motor);
        right_motor_pin.config_ms(20, 1.5 + 0.5 * right_motor);

if __name__ == '__main__':
    rospy.init_node('robot')
    c = motor_control()
    rospy.spin()
