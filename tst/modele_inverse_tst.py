import math
import sys
sys.path.append('../') #pour pouvoir lancer les test depuis tst
sys.path.append('./') #pour pouvoir lancer les test depuis la racine du projet
from src import Modele_inverse

def error(x, y):
    if x == 0:
        return x-y
    return abs(x-y)/x

def test(reel, theo):
    """test si l'erreur relative entre la theorie et le calculé est inferieur à 1%"""
    t = (tuple(map(error, theo, reel)))
    r =(tuple(map(lambda x : x < 0.01, t)))
    return r == (True, True, True)

def all():
    passed = 6
    angle = Modele_inverse.angle(1,1,1)
    if not test(angle.angle_to(3,0,0), (0,math.pi, math.pi)):
        print("failed test 1")
        passed -= 1
    if not test(angle.angle_to(0,3,0), (math.pi/2,math.pi, math.pi)):
        print("failed test 2")
        passed -= 1
    if not test(angle.angle_to(2,0,-1), (0,math.pi, 3*math.pi/2)):
        print("failed test 3")
        passed -= 1
    if not test(angle.angle_to(0,1,-2), (math.pi/2,3*math.pi/2, math.pi)):
        print("failed test 4")
        passed -= 1
    if not test(angle.angle_to(1+math.sqrt(2),0,0), (0,3*math.pi/4, 3*math.pi/2)):
        print("failed test 5")
        passed -= 1
    if not test(angle.angle_to(1+math.sqrt(2)/2,1+math.sqrt(2)/2,0), (math.pi/4,3*math.pi/4, 3*math.pi/2)):
        print("failed test 6")
        passed -= 1
    print("Test passed : {p}/6".format(p=passed))

if __name__ == "__main__":
    all()