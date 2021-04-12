# not empty file
import numpy as np 
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

from constants import constL1, constL2, constL3, LEG_CENTER_POS

def inverse(x, y, z, verbose=True, use_rads=True):
    """
    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    Theta0 = np.arctan2(y,x)

    AH = np.sqrt(x**2+y**2) - constL1
    AC = np.sqrt(AH**2+z**2)
    
    Theta2 = np.pi - np.arccos((constL2**2 + constL3**2 - AC**2) / (2*constL2*constL3))

    var = (AC**2 + constL2**2 - constL3**2) / (2*AC*constL2)
    if -1 <= var <= 1 :
        CAB = np.arccos(var)
    else : 
        return [0., 0., 0.]

    CAH = np.arcsin(z/AC)

    Theta1 = (CAB + CAH)

    return [Theta0, Theta1, Theta2]


def computeIKOriented(pos, leg_id, pos_ini, extra_theta):
    """
    pos : position souhaitée du bout de la patte dans le référentiel du robot centré sur le bout de la patte
    pos_ini : position du bout de la patte au repos/initiale dans le référentiel de la patte centré sur la base de la patte


    - Entrée: positions cibles (tuples (x, y, z)) pour le bout de la patte
    - Sortie: un tableau contenant les positions angulaires cibles (radian) pour les moteurs
    """
    angles = [0, 0, np.pi/2, np.pi, np.pi, -np.pi/2]
    a = [leg_id-1]
    rot = R.from_rotvec(a * np.array([0,0,1])).as_matrix()

    return inverse(*(rot.dot(np.array(pos)) + pos_ini))


def legs(leg1, leg2, leg3, leg4, leg5, leg6):
    """
    - Entrée: positions cibles (tuples (x, y, z)) pour le bout des 6 pattes dans le référentiel du robot
    - Sortie: un tableau contenant les 18 positions angulaires cibles (radian) pour les moteurs
    """

    legs = [leg1, leg2, leg3, leg4, leg5, leg6]
    angles = [0, 0, np.pi/2, np.pi, np.pi, -np.pi/2]
    rots = [R.from_rotvec(a * np.array([0,0,1])).as_matrix() for a in angles]
    
    for i, leg in enumerate(legs):
        leg -= LEG_CENTER_POS[i] # offset
    
    legs = [list(rots[i].dot(np.array(legs[i]))) for i in range(6)] # rotation des référentiels

    for i, leg in enumerate(legs):
        legs[i] = inverse(*leg) # conversion en angles    

    targets = [item for leg in legs for item in leg] # applatie la liste

    return targets


def alKashi(a, b, c):
    angle = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
    if angle > 1:
        angle = 1
    elif angle < -1:
        angle = -1
    return math.acos(angle)


def computeIK(x, y, z, verbose=True, use_rads=True):
    side = 1

    theta1 = math.atan2(y, x)
    AH = math.sqrt(x ** 2 + y ** 2) - constants.constL1
    AM = math.sqrt(AH ** 2 + z ** 2)
    theta3 = side * (np.pi - alKashi(constants.constL2, constants.constL3, AM))

    if (AM == 0):
        theta2 = 0  # Peu importe
    else:
        theta2 = side * alKashi(AM, constants.constL2, constants.constL3) + math.atan2(z, AH)
    return [theta1, theta2,
            theta3]


def computeDK(a, b, c, use_rads=True):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument les angles des moteurs (a, b, c) et retourne la position du bout
    de la patte (x, y, z)

    - Sliders: angles des trois moteurs de la patte
    - Entrée: a, b, c, angles des trois moteurs
    - Sortie: x, y, z, la position du bout de la patte
    """

    T0 = a * constants.THETA1_MOTOR_SIGN  # Theta 1
    T1 = b * constants.THETA2_MOTOR_SIGN - constants.theta2Correction  # Theta 2
    T2 = c * constants.THETA3_MOTOR_SIGN - constants.theta3Correction  # Theta 3

    # point O
    O = np.array([[0], [0], [0]])
    # print("O:", O)

    # point A
    A = np.array([[constants.constL1 * math.cos(T0)], [constants.constL1 * math.sin(T0)], [0]]) + O

    # print("A:", A)

    # point B
    def rotationZ(t):
        return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

    def rotationX(t):
        return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])

    B = np.dot(rotationZ(T0),
               np.array([[constants.constL2 * np.cos(T1 - np.pi)], [0], [constants.constL2 * np.sin(T1 - np.pi)]])) + A

    # print("B:", B)

    # point C
    def rotationY(t):
        return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])

    # point M
    M = np.dot(rotationZ(T0),
               np.dot(rotationY(np.pi - T1), np.array(
                   [[constants.constL3 * np.cos(np.pi - T2)], [0], [constants.constL3 * np.sin(np.pi - T2)]]))) + B

    return [(0,0,0), M]
