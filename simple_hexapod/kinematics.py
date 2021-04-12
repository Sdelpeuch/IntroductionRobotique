import numpy as np
import math
import constants
import matplotlib.pyplot as plt


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
