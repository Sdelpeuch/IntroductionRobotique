import numpy as np
import math
if __package__ is None or __package__ == '':
    import interpolation
else:
    from . import interpolation

l0 = 0.045
l1 = 0.065
l2 = 0.087

def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12
    targets[0] = np.sin(t)

    return targets

def direct(a, b, c):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument les angles des moteurs (a, b, c) et retourne la position du bout
    de la patte (x, y, z)

    - Sliders: angles des trois moteurs de la patte
    - Entrée: a, b, c, angles des trois moteurs
    - Sortie: x, y, z, la position du bout de la patte
    """

    T0 = a # Theta 0
    T1 = np.pi + b # Theta 1
    T2 = c + np.pi # Theta 2

    # point O
    O = np.array([[0], [0], [0]])
    #print("O:", O)

    # point A
    A = np.array([[l0 * math.cos(T0)], [l0 * math.sin(T0)], [0]]) + O
    #print("A:", A)

    # point B
    def rotationZ(t):
        return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0],[0, 0, 1]])

    def rotationX(t):
        return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)],[0, np.sin(t), np.cos(t)]])

    B = np.dot(rotationZ(T0), np.array([[l1 * np.cos(T1 - np.pi)], [0], [l1 * np.sin(T1 - np.pi)]])) + A
    #print("B:", B)

    # point C
    def rotationY(t):
        return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0],[-np.sin(t), 0, np.cos(t)]])

    # point M
    M = np.dot(rotationZ(T0), np.dot(rotationY(np.pi-T1), np.array([[l2 * np.cos(np.pi - T2)], [0], [l2 * np.sin(np.pi - T2)]]))) + B
    # print("M:", M)

    # points = [O, A, B, M]

    # X, Y, Z = np.zeros(4), np.zeros(4), np.zeros(4)
    # for i in range(len(points)):
    #     X[i] = points[i][0]
    #     Y[i] = points[i][1]
    #     Z[i] = points[i][2]

    return [M[0], M[1], M[2]]

def alKashi(a, b, c):
    angle = (a**2 + b**2 - c**2) / (2 * a * b)
    if angle > 1:
        angle = 1
    elif angle < -1:
        angle = -1
    return math.acos(angle)

def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: La position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    side = -1

    theta0 = math.atan2(y, x)
    AH = math.sqrt(x**2 + y**2) - l0
    AM = math.sqrt(AH**2 + z**2)
    theta2 = side * (np.pi - alKashi(l1, l2, AM))

    if (AM == 0):
        theta1 = 0 # Peu importe
    else:
        theta1 = side * alKashi(AM, l1, l2) + math.atan2(z, AH)
    return [theta0, theta1, theta2]

def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    return [0., np.sin(t)*0.3, 0.]

def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    targets = [0]*12

    return targets

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0]*12

    return targets

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")