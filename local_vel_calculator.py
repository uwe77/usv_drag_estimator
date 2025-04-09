#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np

class local_vel_calculator():
    
    def __init__(self):
        rospy.Subscriber('/gazebo/jv/pose', PoseStamped, self.__pose_cb)
        rospy.Subscriber('/jv/sensors/imu/imu/data', Imu, self.__imu_cb)
        self.global_vel = np.zeros(3)
        self.local_vel = np.zeros(3)
        self.heading = 0
        self.angular_vel = np.zeros(3)
        self.last_global_position = np.array([])
        self.last_time = 0
        rospy.Timer(rospy.Duration(0.01), self.cb_publish)

    def __pose_cb(self, data):
        position = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        ])
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9

        self.heading = R.from_quat([
            data.pose.orientation.x, 
            data.pose.orientation.y, 
            data.pose.orientation.z, 
            data.pose.orientation.w
            ]).as_euler('xyz', degrees=False)[2]
        if self.last_global_position.shape[0] == 0:
            self.last_global_position = position
        if self.last_time == 0:
            self.last_time = time
        time_diff = time-self.last_time
        position_diff = position-self.last_global_position
        self.global_vel = position_diff/time_diff

        self.local_vel = np.array([
            self.global_vel[0]*np.cos(self.heading)+self.global_vel[1]*np.sin(self.heading),
            -self.global_vel[0]*np.sin(self.heading)+self.global_vel[1]*np.cos(self.heading),
            self.global_vel[2],
        ])
        self.last_time = time
        self.last_global_position = position

    def __imu_cb(self, data):
        self.angular_vel = np.array([
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        ])

    def cb_publish(self, event):
        output = f" \
        \nglobal:  {self.global_vel} \
        \nlocal :  {self.local_vel} \
        \nangular: {self.angular_vel}"
        rospy.loginfo(output)

if __name__ == '__main__':
    rospy.init_node('local_vel_calculator', anonymous=True)
    node = local_vel_calculator()
    rospy.spin()