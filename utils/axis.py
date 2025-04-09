class axis_drag():

    def __init__(self, name='x'):
        self.name = name
        self.R = 0
        self.RR = 0
        self.U = 0
        self.UU = 0
    
    def get_linear_force(self, v, R, RR):
        """
        calculate drag force
        :param v: (m/s)
        :return:  (N)
        """
        return R * v + RR * v**2
    
    def get_angular_force(self, v, U, UU):
        """
        calculate drag torque
        :param v: (rad/s)
        :return:  (N)
        """
        return U * v + UU * v**2
    
class axis_force():
    
    def __init__(self, name='x', mass=12000, interia=100):
        self.name = name
        self.mass = mass
        self.inertia = interia
    
    def get_linear_force(self, a):
        return a*self.mass
    
    def get_angular_force(self, a):
        return a*self.inertia