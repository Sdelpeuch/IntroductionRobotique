import numpy as np
import matplotlib.pyplot as plt


class LinearSpline:
    def __init__(self, entry=None):
        self.entry = []
        if entry != None:
            self.entry = entry

    def add_entry(self, t, x):
        self.entry.append((t, x))

    def interpolate(self, t):
        x = 0
        for i in range( len(self.entry) -1 ):
            (ti1, xi1) = self.entry[i]
            (ti2, xi2) = self.entry[i+1]
            if t <= ti2 and t >= ti1:
                x = xi1 + (t-ti1)*(xi2-xi1)/(ti2-ti1)
                return x
        return x

class LinearSpline3D:
    def __init__(self):
        self.xentry = []
        self.yentry = []
        self.zentry = []


    def add_entry(self, t, x, y ,z):
        self.xentry.append((t, x))
        self.yentry.append((t, y))
        self.zentry.append((t, z))

    def interpolate(self, t):
        xLinear = LinearSpline(self.xentry)
        yLinear = LinearSpline(self.yentry)
        zLinear = LinearSpline(self.zentry)
        return xLinear.interpolate(t), yLinear.interpolate(t), zLinear.interpolate(t)


if __name__ == "__main__":
    spline = LinearSpline()
    spline.add_entry(0., 0.)
    spline.add_entry(0.5, 0.2)
    spline.add_entry(1.5, -0.4)
    spline.add_entry(2.3, 0.6)
    spline.add_entry(3, 0)

    xs = np.arange(-0.1, 4, 0.1)
    ys = []
    for x in xs:
        ys.append(spline.interpolate(x))

    plt.plot(xs, ys)
    plt.show()
