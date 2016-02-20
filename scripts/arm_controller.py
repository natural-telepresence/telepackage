#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import numpy as np

class GimbalController(object):
	def __init__(self):
		self.pubx = rospy.Publisher("/motor_x/command", Float64, queue_size = 10)
		self.puby = rospy.Publisher("/motor_y/command", Float64, queue_size = 10)
		self.pubz = rospy.Publisher("/motor_z/command", Float64, queue_size = 10)
		self.sub = rospy.Subscriber("/imu/data", Imu, self.imu)

                self.broad = tf.TransformBroadcaster()
                self.first = True
                self.q_zero = (0.0, 0.0, 0.0, 1.0)

        def imu(self, data):

                quaternion = (
                        data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w)

                print(quaternion)

                if self.first:
                        self.first = False
                        self.q_zero = np.copy(quaternion)

                q_error = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.q_zero),quaternion)

                euler = tf.transformations.euler_from_quaternion(q_error)
		roll = -euler[1]
		yaw = -euler[2]
                pitch = -euler[0]

                print("roll, " + str(roll))
                print("pitch, " + str(pitch))
                print("yaw, " + str(yaw))

                self.broad.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(roll, pitch, yaw), rospy.Time.now(), "phone", "world")

                self.pubx.publish(roll + 2.62)

                if (pitch + 2.62) > 1.8 and (pitch + 2.61) < 4.2:
                    self.puby.publish(pitch + 2.62)
		self.pubz.publish(yaw + 2.546)


if __name__ == '__main__':
	try:
		rospy.init_node('controller')
		c = GimbalController()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
