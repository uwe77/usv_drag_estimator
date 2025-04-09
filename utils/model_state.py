from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np
from .axis import axis_force


class axis_force_6dof():
    def __init__(self, mass=12000, inertia=np.array([100, 100, 100])):
        self.mass = mass
        self.inertia = inertia
        self.x = axis_force('x', mass, inertia[0])
        self.y = axis_force('y', mass, inertia[1])
        self.z = axis_force('z', mass, inertia[2])

class model_state():
    
    def __init__(self, mass=12000, inertia=np.array([100, 100, 100])):
        self.position = np.array([])
        self.angular = np.zeros(3)       # Roll, Pitch, Yaw
        self.global_vel = np.zeros(3)
        self.local_vel = np.zeros(3)
        self.angular_vel = np.zeros(3)
        self.global_acc = np.zeros(3)
        self.local_acc = np.zeros(3)
        self.angular_acc = np.zeros(3)
        self.mass = mass
        self.inertia = inertia
        self.axis_force = axis_force_6dof(mass, inertia)
        
        # 上一次 PoseStamped 與 IMU 收到的時間
        self.time = 0
        self.imu_time = 0

    def _check_finite(self, arr_name, arr):
        """
        檢查 arr 是否有 Inf/NaN，如有則打印警告並清零。
        arr_name 只是用來在 warning 中顯示陣列名稱。
        """
        if not np.all(np.isfinite(arr)):
            print(f"[WARN] {arr_name} has Inf or NaN. Reset to 0.")
            return np.zeros_like(arr)
        return arr

    def posestamp_cb(self, data: PoseStamped):
        # (1) get position and orientation (roll, pitch, yaw)
        position = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        ])
        
        roll, pitch, yaw = R.from_quat([
            data.pose.orientation.x, 
            data.pose.orientation.y, 
            data.pose.orientation.z, 
            data.pose.orientation.w
        ]).as_euler('xyz', degrees=False)
        
        self.angular = np.array([roll, pitch, yaw])

        # (2) get time stamp
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9

        # (3) initialization if needed
        if self.position.shape[0] == 0:
            self.position = position
        if self.time == 0:
            self.time = time
        
        # (4) calculate time_diff
        time_diff = time - self.time
        if time_diff < 1e-9:
            # avoid division by 0
            return

        # (5) update global velocity
        position_diff = position - self.position
        global_vel = position_diff / time_diff
        # check if global_vel is finite
        global_g_acc = 9.81*np.array([0, 0, -1])
        self.global_acc = (global_vel - self.global_vel) / time_diff + global_g_acc
        self.global_acc = self._check_finite("global_acc", self.global_acc)
        self.global_vel = global_vel
        self.global_vel = self._check_finite("global_vel", self.global_vel)

        # (6) transform global velocity to local velocity
        cy = np.cos(self.angular[2])
        sy = np.sin(self.angular[2])
        gx, gy, gz = self.global_vel
        self.local_vel = np.array([
            gx * cy + gy * sy,
            -gx * sy + gy * cy,
            gz
        ])
        self.local_vel = self._check_finite("local_vel", self.local_vel)
        # transform global acceleration to local acceleration by yaw
        gx, gy, gz = self.global_acc
        self.local_acc = np.array([
            gx * cy + gy * sy,
            -gx * sy + gy * cy,
            gz
        ])
        self.local_acc = self._check_finite("local_acc", self.local_acc)

        # (7) update last data
        self.time = time
        self.position = position


    def imu_cb(self, data: Imu):
        # (1) calculate current time
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        if self.imu_time == 0:
            self.imu_time = time
        
        time_diff = time - self.imu_time
        if time_diff < 1e-9:
            # avoid division by 0
            return

        # (2) get current IMU angular velocity
        measured_ang_vel = np.array([
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        ])

        # (3) calculate angular acceleration
        self.angular_acc = (measured_ang_vel - self.angular_vel) / time_diff
        self.angular_acc = self._check_finite("angular_acc", self.angular_acc)

        # (4) update old angular_vel
        self.angular_vel = measured_ang_vel
        self.angular_vel = self._check_finite("angular_vel", self.angular_vel)

        # (5) update time
        self.imu_time = time

