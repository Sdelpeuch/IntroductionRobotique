import math

class angle:
    def __init__(self, l0,l1,l2):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2

    def theta_0(self, x, y, z):
        if x == 0:
            return math.pi/2
        return math.atan2(y, x)

    def theta_1(self, x, y, z):
        AM = math.sqrt(z**2 + (math.sqrt(x**2 + y**2) - self.l0**2)**2)
        try :
            a = math.pi + \
                math.asin( - z / AM) - \
                math.acos((self.l1**2 - self.l2**2 +  AM**2) / (2*self.l1 * AM))
        except :
            print("position impossible sur theta_1, returning -1")
            return -1
        return a

    def theta_2(self, x, y, z):
        r = -1
        try: r = 2*math.pi - math.acos((-z**2 - ((x**2+y**2)**(1/2)-self.l0)**2 + self.l2**2 + self.l1**2)/(2*self.l1*self.l2))
        except: print("position impossible sur theta_2, returning -1")
        return r

    def angle_to(self, x, y, z):
        return self.theta_0(x,y,z), self.theta_1(x,y,z), self.theta_2(x,y,z)