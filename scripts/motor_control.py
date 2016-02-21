#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import mraa
import time
import subprocess


class motor_control(object):
    def __init__(self):
        #sub joystick x,y
        self.joy = rospy.Subscriber("/base/cmd_vel", Twist, self.callback)

        self.left_pin_num = rospy.get_param('left_motor_pin', 3)
        #self.left_pin_num = rospy.get_param('left_motor_pin', 60) 
        self.left_motor_pin = mraa.Pwm(60)
        #self.left_motor_pin.period_ms(20)
        #self.left_motor_pin.enable(True)

        self.right_pin_num = rospy.get_param('right_motor_pin', 4)
        #self.right_pin_num = rospy.get_param('right_motor_pin', 62)
        self.right_motor_pin = mraa.Pwm(62)
        #self.right_motor_pin.period_ms(20)
        #self.right_motor_pin.enable(True)

    def callback(self, data):
        fwd_speed = data.linear.x
        turn_speed = data.angular.z
        left_motor = fwd_speed + turn_speed
        right_motor = fwd_speed - turn_speed
        left_motor = self.add_deadband(max(min(left_motor, 1.0), -1.0))
        right_motor = self.add_deadband(max(min(right_motor, 1.0), -1.0))
        
        subprocess.call("echo 20000000 > '/sys/class/pwm/pwm" + str(self.left_pin_num) + "/period_ns'", shell=True)
        subprocess.call("echo " + str(int(1500000 + 500000 * left_motor)) + " > '/sys/class/pwm/pwm" + str(self.left_pin_num) + "/duty_ns'", shell=True)
        subprocess.call("echo 1 > '/sys/class/pwm/pwm" + str(self.left_pin_num) + "/run'", shell=True)
        subprocess.call("echo 20000000 > '/sys/class/pwm/pwm" + str(self.right_pin_num) + "/period_ns'", shell=True)
        subprocess.call("echo " + str(int(1500000 + 500000 * right_motor)) + " > '/sys/class/pwm/pwm" + str(self.right_pin_num) + "/duty_ns'", shell=True)
        subprocess.call("echo 1 > '/sys/class/pwm/pwm" + str(self.right_pin_num) + "/run'", shell=True)

        #print data.linear.x
        #self.left_motor_pin.config_ms(20, 1.5 + 0.5 * left_motor);
        #self.right_motor_pin.config_ms(20, 1.5 + 0.5 * right_motor);

    def signum(self, x): return (x > 0) - (x < 0)

    def add_deadband(self, value):
        if (abs(value) > 0.01):
            return value * 0.5 + self.signum(value) * 0.04
        return value
    
    def stop(self):
        subprocess.call("echo 1500000 > '/sys/class/pwm/pwm" + str(self.left_pin_num) + "/duty_ns'", shell=True)
        subprocess.call("echo 1500000 > '/sys/class/pwm/pwm" + str(self.right_pin_num) + "/duty_ns'", shell=True)

if __name__ == '__main__':
    rospy.init_node('robot')
    c = motor_control()
    rospy.spin()
    c.stop()
