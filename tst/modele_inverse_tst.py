import math
import sys
sys.path.append('../') #pour pouvoir lancer les test depuis tst
sys.path.append('./') #pour pouvoir lancer les test depuis la racine du projet
from src import Modele_inverse

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

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
    position_to_test = [(3,0,0), (0,3,0),
                        (2,0,-1), (0,1,-2),
                        ((1+math.sqrt(2),0,0)), (1+math.sqrt(2)/2,1+math.sqrt(2)/2,0)]
    excepted_result = [(0,math.pi, math.pi), (math.pi/2,math.pi, math.pi),
                       (0,math.pi, 3*math.pi/2), (math.pi/2,3*math.pi/2, math.pi),
                       (0,3*math.pi/4, 3*math.pi/2),(math.pi/4,3*math.pi/4, 3*math.pi/2)]
    print("[TESTING MODELE INVERSE]")
    print("l0 = l1 = l2 = 1")
    for i in range(6):
        print("testing on (x,y,z) = {t}".format(t = position_to_test[i]), end=" -> ")
        x, y, z = position_to_test[i]
        theta_0, theta_1, theta_2 = excepted_result[i]
        if not test(angle.angle_to(x,y,z), (theta_0, theta_1, theta_2)):
            print(bcolors.FAIL + "failed " + bcolors.ENDC)
            passed -= 1
        else:
            print(bcolors.OKGREEN + "passed" + bcolors.ENDC)

    print("Test passed : {p}/6".format(p=passed))


if __name__ == "__main__":
    all()