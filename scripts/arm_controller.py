#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
import numpy as np
from dynamixel_msgs.msg import JointState


class GimbalController(object):
    def __init__(self):
        self.pubx = rospy.Publisher("/motor_x/command", Float64, queue_size=10)
        self.puby = rospy.Publisher("/motor_y/command", Float64, queue_size=10)
        self.pubz = rospy.Publisher("/motor_z/command", Float64, queue_size=10)
        self.sub = rospy.Subscriber("/imu/data", Imu, self.imu)
        self.yaw_sub = rospy.Subscriber("motor_z/state", JointState, self.yaw_cb)
        self.calibration = rospy.Service("/imu/calibrate", Trigger, self.calibrate)

        self.broad = tf.TransformBroadcaster()
        self.first = True
        self.q_zero = (0.0, 0.0, 0.0, 1.0)
        self.current_q = (0.0, 0.0, 0.0, 1.0)

        self.cur_yaw = 0.0

    def yaw_cb(self, data):
        self.cur_yaw = data.current_pos

    def calibrate(self, data):
        self.q_zero = self.current_q
        return [True, 'worked!']

    def imu(self, data):

        self.current_q = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)

        print(self.current_q)

        if self.first:
            self.first = False
            self.q_zero = np.copy(quaternion)

        q_error = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.q_zero), self.current_q)

        euler = tf.transformations.euler_from_quaternion(q_error)
        roll = -euler[1]
        yaw = -euler[2]
        pitch = -euler[0]

        print("roll, " + str(roll))
        print("pitch, " + str(pitch))
        print("yaw, " + str(yaw))

        self.broad.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                                 rospy.Time.now(), "phone", "world")

        self.pubx.publish(roll + 2.62)

        if (pitch + 2.62) > 1.8 and (pitch + 2.61) < 4.2:
            self.puby.publish(pitch + 2.62)

        if self.cur_yaw > 5.2 and yaw < 0.0:
            pass
        elif (self.cur_yaw) < 0.2 and yaw > 0.0:
            pass
        else:
            self.pubz.publish(yaw + 2.546)


if __name__ == '__main__':
    try:
        rospy.init_node('controller')
        c = GimbalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
