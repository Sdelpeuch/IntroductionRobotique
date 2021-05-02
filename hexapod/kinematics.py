import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

import constants


def custom_print(string, to_print=0):
    """
    just a function to print only if to_print = 1
    to_print is 0 by default
    for verbose purpose
    """
    if (to_print == 1):
        print(string)


# # C'est ce qu'on a fait, ça ne marche pas (à comparer pour répondre aux questions du prof)
# def inverse(x, y, z, verbose=True, use_rads=True):
#     """
#     Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
#     (alpha, beta, gamma) pour que la patte atteigne cet objectif

#     - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
#     - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
#     """

#     Theta1 = np.arctan2(y, x) # OK

#     dAP = np.sqrt(x**2 + y**2) - constants.constL1
#     d = np.sqrt(dAP**2 + z**2)
#     a = np.arctan2(z, dAP)

#     # --> AlKashi sur le 2eme angle
#     angle2 = (constants.constL2**2 + d**2 - constants.constL3**2) / (2 * constants.constL2 * d)
#     if angle2 > 1:
#         angle2 = 1
#     elif angle2 < -1:
#         angle2 = -1
#     b = np.arccos( angle2 )

#     Theta2 = a + b + constants.theta2Correction # OK
    
#     # --> AlKashi sur le 3eme angle
#     angle3 = (constants.constL2**2 + constants.constL3**2 - d**2) / (2 * constants.constL2**2 * constants.constL3**2)
#     if angle3 > 1:
#         angle3 = 1
#     elif angle3 < -1:
#         angle3 = -1
#     Theta3 = np.pi - np.arccos( angle3 )
    
#     Theta3 += constants.theta3Correction

#     print('theta3:', Theta3, ', angle3:', angle3)

#     return [Theta1, -Theta2, Theta3]

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


def alKashi(a, b, c, sign=-1):
    """
     A
     |     b
    c|
     |__________
     B    a     C
    Pour un triangle de longueurs a, b, c, donne l'angle BCA
    Le domaine de définition d'arcccos [-1, 1] est respecté.
    """
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))

def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)

# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


# Takes an angle that's between 0 and 2pi and returns an angle that is between -pi and pi
def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes
# separated by the distances (l1, l2, l3), returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constants.constL1,
    l2=constants.constL2,
    l3=constants.constL3,
    verbose=False,
    use_rads=constants.USE_RADS_OUTPUT,
    sign=-1,
    use_mm=constants.USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - constants.Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(constants.THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                constants.THETA2_MOTOR_SIGN * (theta2 + constants.theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                constants.THETA3_MOTOR_SIGN * (theta3 + constants.theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(constants.THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                constants.THETA2_MOTOR_SIGN * (math.degrees(theta2) + constants.theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                constants.THETA3_MOTOR_SIGN * (math.degrees(theta3) + constants.theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        custom_print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x,
                y,
                z,
                result[0],
                result[1],
                result[2],
            )
        )

    return result

def computeIKNous(x, y, z, verbose=True, use_rads=True):
    # return [0,
    #         0 + constants.THETA2_MOTOR_SIGN * constants.theta2Correction,
    #         0 + constants.THETA3_MOTOR_SIGN * constants.theta3Correction]
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

    return inverse(x, y, z)

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


"""[dictionnaire des legs]

dict_items([(1, [id: 11, goal_position: 0, present_position: 0,
                 id: 12, goal_position: 0, present_position: 0,
                 id: 13, goal_position: 0, present_position: 0]),
            (2, [id: 21, goal_position: 0, present_position: 0,
                 id: 22, goal_position: 0, present_position: 0,
                 id: 23, goal_position: 0, present_position: 0]),
            (3, [id: 31, goal_position: 0, present_position: 0,
                 id: 32, goal_position: 0, present_position: 0,
                 id: 33, goal_position: 0, present_position: 0]),
            (4, [id: 41, goal_position: 0, present_position: 0,
                 id: 42, goal_position: 0, present_position: 0,
                 id: 43, goal_position: 0, present_position: 0]),
            (5, [id: 51, goal_position: 0, present_position: 0,
                 id: 52, goal_position: 0, present_position: 0,
                 id: 53, goal_position: 0, present_position: 0]),
            (6, [id: 61, goal_position: 0, present_position: 0,
                 id: 62, goal_position: 0, present_position: 0,
                 id: 63, goal_position: 0, present_position: 0])])
"""

def setPositionToRobotLeg(Theta1, Theta2, Theta3, leg, robot):
    # modifie les consignes du robot pour une jambe
    custom_print(robot.legs.items()) # Avant
    for v in robot.legs[leg]:
        v[0].goal_position = Theta1
        v[1].goal_position = Theta2
        v[2].goal_position = Theta3
    custom_print(robot.legs.items()) # Après

def setPositionToRobot(robot, params):
    # Délai entre deux consignes pour le robot, en secondes
    dtime = 3

    # On modifie les consignes pour les angles des pattes
    for key, value in robot.legs.items():
        # lire la valeur des Thetas dans le tableau.
        # Theta1, Theta2, Theta3 =
        setPositionToRobotLeg(Theta1, Theta2, Theta3, key, robot)

    # On envoie la consigne au robot
    robot.smooth_tick_read_and_write(delay=dtime, verbose=False)

def computeIKOriented(x, y, z, leg_id, params, verbose=False, extra_angle = 0):
    """
    pos : position souhaitée du bout de la patte dans le référentiel du robot centré sur le bout de la patte
    pos_ini : position du bout de la patte au repos/initiale dans le référentiel de la patte centré sur la base de la patte


    - Entrée: positions cibles (tuples (x, y, z)) pour le bout de la patte
    - Sortie: un tableau contenant les positions angulaires cibles (radian) pour les moteurs
    """
    pos = (x, y, z*constants.Z_DIRECTION)

    angles = constants.LEG_ANGLES # [np.pi/4, 0, np.pi/2, np.pi, np.pi, -np.pi/2]
    a = angles[leg_id-1] + extra_angle
    rot = R.from_rotvec(a * np.array([0,0,1])).as_matrix()

    pos_ini = params.initLeg[leg_id-1] + [params.z]

    if verbose:
        print(np.array(pos))
        print(np.array(pos_ini))
        print(np.array(pos) + np.array(pos_ini))

    res = rot.dot(np.array(pos)) + np.array(pos_ini)
    return computeIK(*res)

def walkDistanceAngle(dist, angle, step_dist, step_height, params):
    """
    Retourne un tableau contenant une successions des positions clefs des 18 angles 
    des steppers permettant la marche sur la distance dist avec un angle donné
    """
    res = []
    nb_step = int(dist//step_dist)

    for _ in range(nb_step-1):
        calculated_angles = []
        for i in range(3):
            calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
            calculated_angles += computeIKOriented(0, 0, step_height, i*2+2, params, extra_angle=angle)
        res += [calculated_angles]
        calculated_angles = []
        for i in range(3):
            calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
            calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
        res += [calculated_angles]
        calculated_angles = []
        for i in range(3):
            calculated_angles += computeIKOriented(0, 0, step_height, i*2+1, params, extra_angle=angle)
            calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
        res += [calculated_angles]
        calculated_angles = []
        for i in range(3):
            calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
            calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
        res += [calculated_angles]

    return res


def oldWalkDistanceAngle(dist, angle, step_dist, step_height, params):
    """
    Retourne un tableau contenant une successions des positions clefs des 18 angles 
    des steppers permettant la marche sur la distance dist avec un angle donné
    """
    res = []
    res += [toIniPos(params)]

    nb_step = int(dist//step_dist)

    if(nb_step != 0):
        # first half-step
        # calculated_angles = []
        # for i in range(3):
        #     calculated_angles += computeIKOriented(step_dist/4, 0, step_height, i*2+1, params, extra_angle=angle)
        #     calculated_angles += computeIKOriented(-step_dist/4, 0, 0, i*2+2, params, extra_angle=angle)
        # res += [calculated_angles]
        # calculated_angles = []
        # for i in range(3):
        #     calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
        #     calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
        # res += [calculated_angles]

        # steps (nb_steps-1)
        for j in range(nb_step-1):
            calculated_angles = []
            for i in range(3):
                calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
                calculated_angles += computeIKOriented(0, 0, step_height, i*2+2, params, extra_angle=angle)
            res += [calculated_angles]
            calculated_angles = []
            for i in range(3):
                calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
                calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
            res += [calculated_angles]
            calculated_angles = []
            for i in range(3):
                calculated_angles += computeIKOriented(0, 0, step_height, i*2+1, params, extra_angle=angle)
                calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
            res += [calculated_angles]
            calculated_angles = []
            for i in range(3):
                calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
                calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
            res += [calculated_angles]

        # last half-step
        # calculated_angles = []
        # for i in range(3):
        #     calculated_angles += computeIKOriented(step_dist/4, 0, 0, i*2+1, params, extra_angle=angle)
        #     calculated_angles += computeIKOriented(-step_dist/4, 0, step_height, i*2+2, params, extra_angle=angle)
        # res += [calculated_angles]
        # calculated_angles = []
        # for i in range(3):
        #     calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
        #     calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
        # res += [calculated_angles]

    #reste 
    # if (reste > eps):
    #     calculated_angles = []
    #     for i in range(3):
    #         calculated_angles += computeIKOriented(reste/4, 0, step_height, i*2+1, params, extra_angle=angle)
    #         calculated_angles += computeIKOriented(-reste/4, 0, 0, i*2+2, params, extra_angle=angle)
    #     res += [calculated_angles]
    #     calculated_angles = []
    #     for i in range(3):
    #         calculated_angles += computeIKOriented(reste/2, 0, 0, i*2+1, params, extra_angle=angle)
    #         calculated_angles += computeIKOriented(-reste/2, 0, 0, i*2+2, params, extra_angle=angle)
    #     res += [calculated_angles]
    #     calculated_angles = []
    #     for i in range(3):
    #         calculated_angles += computeIKOriented(reste/4, 0, 0, i*2+1, params, extra_angle=angle)
    #         calculated_angles += computeIKOriented(-reste/4, 0, step_height, i*2+2, params, extra_angle=angle)
    #     res += [calculated_angles]
    #     calculated_angles = []
    #     for i in range(3):
    #         calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
    #         calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
    #     res += [calculated_angles]
    
    return res

def walkXY(x_dist, y_dist, step_dist, step_height, params):
    """
    Retourne un tableau contenant une successions des positions clefs des 18 angles 
    des steppers permettant de marcher jusqu'à la position (x,y)
    """
    distance = np.sqrt(x_dist**2+y_dist**2)
    angle = math.atan2(y_dist, x_dist)
    return walkDistanceAngle(distance, angle, step_dist, params)

def toIniPos(params):
    res = []
    for i in range(1, 7):
        res += computeIKOriented(0, 0, 0, i, params)
    return res


def make_smooth(new_angles, last_angles, smooth_num=25):
    smooth = []
    for i in range(smooth_num):
        new = []
        for index in range(0, len(new_angles)):
            to_add = (-last_angles[index]+new_angles[index])/smooth_num
            new += [last_angles[index]+to_add*i]
        smooth += [new]
    return smooth


def compute_pos_jump(period, dt):
    # Init, flexion, exension, init
    time = [0, 0.3, 0.4, 0.58] # in %
    x = [0, 0, 0, 0]
    y = [0, 0, 0, 0]
    z = [-0.06, 0.03, -0.3, -0.06]
    
    lenght = int(1/dt)
    # time
    t = [e * period * lenght for e in time]
    X = []
    Y = []
    Z = []

    step = 0
    for i in range(1, lenght+1):
        if step+1 < len(t) and t[step+1] <= i:
            step += 1
        X.append(x[step])
        Y.append(y[step])
        Z.append(z[step])
    return X, Y, Z

def jump(params):
    # period en secondes, laisser à 1.5s
    period = 1.5
    dt = .1/period
    X, Y, Z = compute_pos_jump(period, dt)

    steps = []
    for i in range(len(X)):
        step = []
        for leg in range(1, 7):
            step += computeIKOriented(X[i], Y[i], Z[i], leg, params, 0)
        steps.append(step)
    
    # print
    if False:
        for v in steps:
            print("1:{:.2f} {:.2f} {:.2f}, 2:{:.2f} {:.2f} {:.2f}, 3:{:.2f} {:.2f} {:.2f}, 4:{:.2f} {:.2f} {:.2f}, 5:{:.2f} {:.2f} {:.2f}, 6:{:.2f} {:.2f} {:.2f}".format(
                v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12],
                v[13], v[14], v[15], v[16], v[17]))
    return steps  
