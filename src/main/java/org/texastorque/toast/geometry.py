from scipy.spatial.transform import Rotation as R

class Pose3d:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    @staticmethod
    def from_matrix(pose):
        return Pose3d(pose[0][0], pose[1][0], pose[2, 0])
    def to_list(self):
        return [self.x, self.y, self.z]
    def to_str(self, prec=2):
        return str((round(self.x, prec), round(self.y, prec), round(self.z, prec)))
 
    def __str__(self):
        return self.to_str()

class Rot3d:
    def __init__(self, r):
        self.r = r
    @staticmethod
    def from_matrix(rot):
        return Rot3d(R.from_matrix(rot))

    def to_euler(self, deg=False):
        return self.r.as_euler("xyz", degrees=deg)
    def x(self, deg=False):
        return self.to_euler(deg=deg)[0]
    def y(self, deg=False):
        return self.to_euler(deg=deg)[1]
    def z(self, deg=False):
        return self.to_euler(deg=deg)[2]

    def to_quaternion(self):
        return self.r.as_quat()
    def qw(self):
        return self.to_quaternion()[0]
    def qx(self):
        return self.to_quaternion()[1]
    def qy(self):
        return self.to_quaternion()[2]
    def qz(self):
        return self.to_quaternion()[3]


    def to_str(self, prec=2, deg=False):
        return str((round(self.x(deg=deg), prec), round(self.y(deg=deg), prec), round(self.z(deg=deg), prec)))
    def __str__(self):
        return self.to_str()

