#!/usr/bin/env python
import rospy
import tf.transformations
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class GimbalController(object):
	def __init__(self):
		self.pubx = rospy.Publisher("/motor_x/command", Float64, queue_size = 10)
		self.puby = rospy.Publisher("/motor_y/command", Float64, queue_size = 10)
		self.pubz = rospy.Publisher("/motor_z/command", Float64, queue_size = 10)
		self.sub = rospy.Subscriber("/cardboard/imu", Imu, self.imu)

	def imu(self, data):
                quaternion = (
                        data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w)

                euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		self.pubx.publish(roll + 2.62)
		self.puby.publish(pitch + 2.62)
		self.pubz.publish(yaw + 2.546)


if __name__ == '__main__':
	try:
		rospy.init_node('controller')
		c = GimbalController()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
