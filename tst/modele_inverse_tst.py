import math
import sys
sys.path.append('../')

from src import Modele_inverse
def error(x, y):
    if x == 0:
        return x-y
    return abs(x-y)/x

def test(reel, theo):
    """test si l'error relative entre la theorie et le calculé est inferieur à 1%"""
    t = (tuple(map(error, theo, reel)))
    r =(tuple(map(lambda x : x < 0.01, t)))
    print(r)
    return r == (True, True, True)

def all():
    angle = Modele_inverse.angle(1,1,1)
    if not test(angle.angle_to(3,0,0), (0,math.pi, math.pi)):
        print("failed test 1")
    if not test(angle.angle_to(0,3,0), (math.pi/2,math.pi, math.pi)):
        print("failed test 2")
    if not test(angle.angle_to(2,0,-1), (0,math.pi, 3*math.pi/2)):
        print("failed test 3")
    if not test(angle.angle_to(0,1,-2), (math.pi/2,3*math.pi/2, math.pi)):
        print("failed test 4")
    if not test(angle.angle_to(1+math.sqrt(2),0,0), (0,3*math.pi/2, 3*math.pi/2)):
        print("failed test 5")
    if not test(angle.angle_to(1+math.sqrt(2),1+math.sqrt(2),0), (math.pi/4,3*math.pi/2, 3*math.pi/2)):
        print("failed test 6")

if __name__ == "__main__":
    all()