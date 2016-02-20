#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu 

def motor(data):
    pass

def controller():
    rospy.init_node('controller')
    pub1 = rospy.Publisher("/motor_x/command", Float64, queue size=10)
    pub2 = rospy.Publisher("/motor_y/command", Float64, queue size=10)
    pub3 = rospy.Publisher("/motor_z/command", Float64, queue size=10)
    sub = rospy.Subscriber("/cardboard/imu", Imu, motor) 

    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
		pass
