import numpy as np
import math
import matplotlib.pyplot as plt

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
    targets = [0] * 12
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

    T0 = a  # Theta 0
    T1 = np.pi + b  # Theta 1
    T2 = c + np.pi  # Theta 2

    # point O
    O = np.array([[0], [0], [0]])
    # print("O:", O)

    # point A
    A = np.array([[l0 * math.cos(T0)], [l0 * math.sin(T0)], [0]]) + O

    # print("A:", A)

    # point B
    def rotationZ(t):
        return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

    def rotationX(t):
        return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])

    B = np.dot(rotationZ(T0), np.array([[l1 * np.cos(T1 - np.pi)], [0], [l1 * np.sin(T1 - np.pi)]])) + A

    # print("B:", B)

    # point C
    def rotationY(t):
        return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])

    # point M
    M = np.dot(rotationZ(T0),
               np.dot(rotationY(np.pi - T1), np.array([[l2 * np.cos(np.pi - T2)], [0], [l2 * np.sin(np.pi - T2)]]))) + B
    # print("M:", M)

    # points = [O, A, B, M]

    # X, Y, Z = np.zeros(4), np.zeros(4), np.zeros(4)
    # for i in range(len(points)):
    #     X[i] = points[i][0]
    #     Y[i] = points[i][1]
    #     Z[i] = points[i][2]

    return [M[0], M[1], M[2]]


def alKashi(a, b, c):
    angle = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
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

    side = 1

    theta0 = math.atan2(y, x)
    AH = math.sqrt(x ** 2 + y ** 2) - l0
    AM = math.sqrt(AH ** 2 + z ** 2)
    theta2 = side * (np.pi - alKashi(l1, l2, AM))

    if (AM == 0):
        theta1 = 0  # Peu importe
    else:
        theta1 = side * alKashi(AM, l1, l2) + math.atan2(z, AH)
    return [theta0, theta1, theta2]


def draw(t):
    interpolator = interpolation.LinearSpline3D()
    interpolator.add_entry(0, 0.05, 0, 0)
    interpolator.add_entry(1, 0.05, 0, 0.1)
    interpolator.add_entry(2, 0.05, 0.1, 0.1)
    interpolator.add_entry(3, 0.05, 0, 0)
    x, y, z = interpolator.interpolate(t % 3)

    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    return inverse(x, y, z)


def step(t, patte_num, speed_x, speed_y):
    d = 0.120
    z0 = -0.05

    initiale = [(-d, d, z0), (-d, -d, z0), (d, -d, z0), (d, d, z0)]
    high = (initiale[patte_num][0] + speed_x / 2, initiale[patte_num][1] + speed_y / 2, initiale[patte_num][2] + 0.05)
    end = (initiale[patte_num][0] + speed_x / 2, initiale[patte_num][1] + speed_y / 2, initiale[patte_num][2])

    interpolator = interpolation.LinearSpline3D()

    interpolator.add_entry(0, * initiale[patte_num])
    interpolator.add_entry(1, * high)
    interpolator.add_entry(2, * end)
    interpolator.add_entry(3, * initiale[patte_num])
    x, y, z = interpolator.interpolate(t % 3)

    print(t)

    return x, y, z


def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0] * 12
    offset = math.sqrt(0.040 * 0.040 / 2)

    def rotation(t, x, y):
        return np.dot(np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]), np.array([[x], [y]]))

    def inverse_tuple_rotation_translation(my_tuple, t, x, y):
        x_ = my_tuple[0]
        y_ = my_tuple[1]
        z_ = my_tuple[2]

        # translation
        x_ += x * offset
        y_ += y * offset

        # rotation
        [x_, y_] = rotation(t, x_, y_)

        return inverse(x_, y_, z_)

    # Patte 1 +x,+y
    temp_x, temp_y, temp_z = inverse_tuple_rotation_translation(leg1, - 3 * np.pi / 4, 1, -1)
    targets[0:2] = [temp_x, temp_y, temp_z]
    # Patte 2 +x,+y
    temp_x, temp_y, temp_z = inverse_tuple_rotation_translation(leg2, 3 * np.pi / 4, 1, 1)
    targets[3:5] = [temp_x, temp_y, temp_z]
    # Patte 3 +x,+y
    temp_x, temp_y, temp_z = inverse_tuple_rotation_translation(leg3, np.pi / 4, -1, 1)
    targets[6:8] = [temp_x, temp_y, temp_z]
    # Patte 4 +x,+y
    temp_x, temp_y, temp_z = inverse_tuple_rotation_translation(leg4, -np.pi / 4, -1, -1)
    targets[9:] = [temp_x, temp_y, temp_z]

    return targets

leg1 = []

def walk_2(t, speed_x, speed_y, speed_rotation):

    # Déterminer la position des pattes en fonction de t
    # print("avancer_x = {0}\navancer_y = {1}".format(speed_x, speed_y))
    patte1 = step(t+1, 0, speed_x, speed_y)
    patte2 = step(t, 1, speed_x, speed_y)
    patte3 = step(t+1, 2, speed_x, speed_y)
    patte4 = step(t, 3, speed_x, speed_y)

    targets = legs(patte1, patte2, patte3, patte4)


    if False:
        x1, y1, z1 = patte3
        leg1.append([x1, y1, z1])
        tmp = np.array(leg1)
        plt.clf()
        plt.plot(tmp.T[0], color = 'red')
        plt.plot(tmp.T[1], color='green')
        plt.plot(tmp.T[2], color='blue')
        plt.grid()
        plt.pause(0.01)

    return targets


def walk(t, speed_x, speed_y, speed_rotation):
    return walk_2(t, speed_x, speed_y, speed_rotation)
    d = 0.120
    z0 = -0.05
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0] * 12
    # Position initiale
    initiale = [(-d, d, z0), (-d, -d, z0), (d, -d, z0), (d, d, z0)]

    if t == 0:
        return legs(initiale[0], initiale[1], initiale[2], initiale[3])

    # Déterminer la position des pattes en fonction de t
    avancer_x  = speed_x
    avancer_y = speed_y
    print("avancer_x = {0}\n avancer_y = {1}".format(avancer_x, avancer_y))
    avancer_z = 0
    if speed_x != 0 or speed_y != 0:
        avancer_z = 0.03
    step = int((t * 15) % 4)
    # print("t : {0}".format(t))

    if step == 0:
        print("step {0}".format(step))
        patte1 = (initiale[0][0] - avancer_x, initiale[0][1] - avancer_y, initiale[0][2] + 0)
        patte2 = (initiale[1][0] + avancer_x / 2, initiale[1][1] + avancer_y / 2, initiale[1][2])
        patte3 = (initiale[2][0] - avancer_x, initiale[2][1] - avancer_y, initiale[2][2] + 0)
        patte4 = (initiale[3][0] + avancer_x / 2, initiale[3][1] + avancer_y / 2, initiale[3][2])

    elif step == 1:
        print("step {0}".format(step))
        patte1 = (initiale[0][0] + avancer_x / 2, initiale[0][1] + avancer_y / 2, initiale[0][2] + avancer_z)
        patte2 = initiale[1]
        patte3 = (initiale[2][0] + avancer_x / 2, initiale[2][1] + avancer_y / 2, initiale[2][2] + avancer_z)
        patte4 = initiale[3]


    elif step == 2:
        # print("step {0}".format(step))
        patte1 = (initiale[0][0] + avancer_x / 2, initiale[0][1] + avancer_y / 2, initiale[0][2])
        patte2 = (initiale[1][0] - avancer_x, initiale[1][1] - avancer_y, initiale[1][2])
        patte3 = (initiale[2][0] + avancer_x / 2, initiale[2][1] + avancer_y / 2, initiale[2][2])
        patte4 = (initiale[3][0] - avancer_x, initiale[3][1] - avancer_y, initiale[3][2])


    else:  # step == 3
        # print("step {0}".format(step))
        patte1 = initiale[0]
        patte2 = (initiale[1][0] + avancer_x / 2, initiale[1][1] + avancer_y / 2, initiale[1][2] + avancer_z)
        patte3 = initiale[2]
        patte4 = (initiale[3][0] + avancer_x / 2, initiale[3][1] + avancer_y / 2, initiale[3][2] + avancer_z)

    if False:
        x1, y1, z1 = patte1
        leg1.append([x1, y1, z1])
        tmp = np.array(leg1)
        plt.clf()
        plt.plot(tmp.T[0], color = 'red')
        plt.plot(tmp.T[1], color='green')
        plt.plot(tmp.T[2], color='blue')
        plt.grid()
        plt.pause(0.05)

    targets = legs(patte1, patte2, patte3, patte4)
    # print('\np1:', patte1, '\np2:', patte2, '\np3:', patte3, '\np4:', patte4)
    return targets


if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")
