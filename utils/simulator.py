from .axis import axis_drag
from .model_state import model_state
import numpy as np
from scipy.spatial.transform import Rotation as R


class axis_drag_6dof():
    def __init__(self):
        self.x = axis_drag('x')
        self.y = axis_drag('y')
        self.z = axis_drag('z')

class Simulator():
    def __init__(self, mass=2600, inertia=np.array([2000.0, 10000.0, 10000.0])):
        self.veh = model_state(mass=mass, inertia=inertia)
        self.axis_drag = axis_drag_6dof()

    def rotate_local_to_global_only_xy(self, vec_local):
        """
        Rotate local vector into global frame using full (roll, pitch, yaw).
        """
        roll, pitch, yaw = self.veh.angular  # already in radians
        rotation = R.from_euler('xyz', [roll, pitch, 0], degrees=False)
        vec_global = rotation.apply(vec_local)
        return vec_global
