import numpy as np
import matplotlib.pyplot as plt


class LinearSpline:
    def __init__(self):
        pass

    def add_entry(self, t, x):
        pass

    def interpolate(self, t):
        return 0.


class LinearSpline3D:
    def __init__(self):
        pass

    def add_entry(self, t, x, y ,z):
        pass

    def interpolate(self, t):
        return 0., 0., 0.


if __name__ == "__main__":
    spline = LinearSpline()
    spline.add_entry(0., 0.)
    spline.add_entry(0.5, 0.2)
    spline.add_entry(1.5, -0.4)
    spline.add_entry(2.3, 0.6)

    xs = np.arange(-0.1, 2.5, 0.1)
    ys = []
    for x in xs:
        ys.append(spline.interpolate(x))

    plt.plot(xs, ys)
    plt.show()
