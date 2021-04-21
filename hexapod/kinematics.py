# not empty file
import numpy as np 
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

import constants


def inverse(x, y, z, verbose=True, use_rads=True):
    """
    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    Theta0 = np.arctan2(y,x)

    AH = np.sqrt(x**2 + y**2) - constants.constL1
    AC = np.sqrt(AH**2 + z**2)
    
    Theta2 = np.pi - np.arccos((constants.constL2**2 + constants.constL3**2 - AC**2) / (2 * constants.constL2 * constants.constL3))

    var = (AC**2 + constants.constL2**2 - constants.constL3**2) / (2 * AC * constants.constL2)
    if -1 <= var <= 1 :
        CAB = np.arccos(var)
    else : 
        return [0., 0., 0.]

    CAH = np.arcsin( z/AC )

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
    rots = [R.from_rotvec(a * np.array([0, 0, 1])).as_matrix() for a in angles]
    
    for i, leg in enumerate(legs):
        leg -= LEG_CENTER_POS[i] # offset
    
    legs = [list(rots[i].dot(np.array(legs[i]))) for i in range(6)] # rotation des référentiels

    for i, leg in enumerate(legs):
        legs[i] = inverse(*leg) # conversion en angles    

    targets = [item for leg in legs for item in leg] # applatie la liste

    return targets


def alKashi(a, b, c):
    """
     A
     |     b
    c|
     |__________
     B    a     C
    Pour un triangle de longueurs a, b, c, donne l'angle BCA
    """
    angle = (a**2 + b**2 - c**2) / (2 * a * b)
    if angle > 1:
        angle = 1
    elif angle < -1:
        angle = -1
    return math.acos(angle)

def computeIK_test(x, y, z, verbose=True, use_rads=True):
    z = constants.Z_DIRECTION * z
    theta1 = constants.THETA1_MOTOR_SIGN * math.atan2(y, x) # OK
    AH = math.sqrt(x ** 2 + y ** 2) - constants.constL1
    AM = math.sqrt(AH ** 2 + z ** 2)
    theta2 = math.atan2(z, AH) - alKashi(constants.constL2, AM, constants.constL3)
    theta3 = math.atan2(AH, z) - alKashi(constants.constL3, AM, constants.constL2)
    return [theta1*constants.THETA1_MOTOR_SIGN,
            theta2 - constants.THETA2_MOTOR_SIGN * constants.theta2Correction,
            theta3 + constants.THETA3_MOTOR_SIGN * constants.theta3Correction]

def computeIK(x, y, z, verbose=True, use_rads=True):
    # return [0,
    #         0 + constants.THETA2_MOTOR_SIGN * constants.theta2Correction,
    #         0 + constants.THETA3_MOTOR_SIGN * constants.theta3Correction]
    return computeIK_test(0.26, 0, 0, verbose, use_rads)
    z = constants.Z_DIRECTION * z
    # T2    t - pi, t + pi, pi - t
    # T3    t - pi, t + pi, pi - t

    theta1 = constants.THETA1_MOTOR_SIGN * math.atan2(y, x) # OK

    AH = math.sqrt(x**2 + y**2) - constants.constL1
    AM = math.sqrt(AH**2 + z**2)

    theta2 = constants.THETA2_MOTOR_SIGN * alKashi(AM, constants.constL2, constants.constL3) + math.atan2(z, AH)
    

    theta3 = constants.THETA3_MOTOR_SIGN * (alKashi(constants.constL3, constants.constL2, AM)) 


    if False: #postion de debug
        return [0, constants.THETA2_MOTOR_SIGN * constants.theta2Correction, constants.THETA3_MOTOR_SIGN * constants.theta3Correction] # bras allongé = 0, t2corr, -t3corr


    return [theta1, theta2 + constants.theta2Correction, theta3 - constants.theta3Correction]


def computeDKDetailed(a, b, c, use_rads=True):
    """
    python sim_hexa.py -m frozen-direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument les angles des moteurs (a, b, c) et retourne la position du bout
    de la patte (x, y, z)
    
    L'offset de l'épaule se fera après cette fonction.

    - Sliders: angles des trois moteurs de la patte
    - Entrée: a, b, c, angles des trois moteurs
    - Sortie: [x, y, z] x 4 les positions des points O, A, B et M
    """

    T1 = a * constants.THETA1_MOTOR_SIGN  # Theta 1
    T2 = b * constants.THETA2_MOTOR_SIGN - constants.theta2Correction  # Theta 2
    T3 = c * constants.THETA3_MOTOR_SIGN - constants.theta3Correction  # Theta 3

    # point Origine
    Origine = np.array([[0], [0], [0]])
    # print("O:", O)

    # point A
    A = np.array([[constants.constL1 * math.cos(T1)], [constants.constL1 * math.sin(T1)], [0]]) + Origine
    #print("A:", A)


    # point B
    def rotationZ(t):
        return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

    B = np.dot(rotationZ(T1), np.array([[constants.constL2 * np.cos(-T2)], [0], [constants.constL2 * np.sin(-T2)]])) + A
    #print("B:", B)


    def rotationY(t):
        return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])

    # point M
    M = np.dot(rotationZ(T1), np.dot(rotationY(T2 + np.pi),
        np.array([[constants.constL3 * np.cos(np.pi - T3)], [0], [constants.constL3 * np.sin(np.pi - T3)]]))) + B
    #print("M:", M)

    def flat(regular_list):
        return [item for sublist in regular_list for item in sublist]

    O_flat = flat(Origine)
    A_flat = flat(A)
    B_flat = flat(B)
    M_flat = flat(M)
    return [O_flat, A_flat, B_flat, M_flat]

def computeDK(a, b, c, use_rads=True):
    """
    Ne retourne que la postion du point M, le bout de la patte.
    Cf computeDKDetailed pour plus de détails
    """
    return computeDKDetailed(a, b, c, use_rads)[-1]


def rotaton_2D(x, y, z, leg_angle):
    def rotation2D(angle):
        return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    xy = np.array([[x], [y]])
    xy = np.dot(rotation2D(leg_angle), xy)
    return [xy[0], xy[1], z]