#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import mraa
import time


class motor_control(object):
    def __init__(self):
        #sub joystick x,y
        self.joy = rospy.Subscriber("/controller/input", Twist, self.callback)

        left_motor_pin = mraa.Pwm(3)
        left_motor_pin.period_us(20000)
        left_motor_pin.enable(True)

        right_motor_pin = mraa.Pwm(4)
        right_motor_pin.period_us(20000)
        right_motor_pin.enable(True)

    def callback(self, data):
        fwd_speed = data.linear.x
        turn_speed = data.angular.z
        left_motor = fwd_speed + turn_speed
        right_motor = fwd_speed - turn_speed

        left_motor_pin.write(((0.5*(left_motor+1))+1)*0.0005)
        right_motor_pin.write(((0.5*(right_motor+1))+1)*0.0005)



if __name__ == '__main__':
    rospy.init_node('robot')
    c = motor_control()
    rospy.spin()
