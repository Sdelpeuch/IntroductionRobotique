import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

import constants


def custom_print(string, to_print=0):
    """
    Just a function to print only if to_print = 1
    to_print is 0 by default
    For verbose purpose
    """
    if (to_print == 1):
        print(string)


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
        return 0
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))

def angleRestrict(angle, use_rads=False):
    # if angle is in radians, gives it in the domain [-pi; +pi]
    # if angle is in degrees, gives it in the domain [-180; +180]
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


def rotate(angle, max_step_dist, step_height, params, l_corner = 0.21, l_side = 0.18):
    res = []

    #perimeters
    l_leg = params.initLeg[0][0]
    R_side = l_side + l_leg
    R_corner = l_corner + l_leg
    P_side = math.pi * 2 * R_side
    P_corner = math.pi * 2 * R_corner

    #determine steps distances
    nb_step = (int)((P_corner * angle / 2*math.pi)// max_step_dist) + 1
    step_dist = (P_corner * angle / 2*math.pi) / nb_step
    step_corner = step_dist
    step_side = P_side / P_corner * step_dist

    for _ in range(nb_step):
        calculated_angles = []
        calculated_angles += computeIK(l_corner, 0, params.z + step_height)
        calculated_angles += computeIK(l_corner, 0, params.z)
        calculated_angles += computeIK(l_side, 0, params.z + step_height)
        calculated_angles += computeIK(l_corner, 0, params.z)
        calculated_angles += computeIK(l_corner, 0, params.z + step_height)
        calculated_angles += computeIK(l_side, 0, params.z)
        res += [calculated_angles]

        calculated_angles = []
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, -math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_side/2)*R_side - l_side, math.sin(step_side/2)*R_side, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, -math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_side/2)*R_side - l_side, -math.sin(step_side/2)*R_side, params.z)
        res += [calculated_angles]

        calculated_angles = []
        calculated_angles += computeIK(l_corner, 0, params.z)
        calculated_angles += computeIK(l_corner, 0, params.z + step_height)
        calculated_angles += computeIK(l_side, 0, params.z)
        calculated_angles += computeIK(l_corner, 0, params.z + step_height)
        calculated_angles += computeIK(l_corner, 0, params.z)
        calculated_angles += computeIK(l_side, 0, params.z + step_height)
        res += [calculated_angles]

        calculated_angles = []
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, -math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_side/2)*R_side - l_side, -math.sin(step_side/2)*R_side, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_corner/2)*R_corner - l_corner, -math.sin(step_corner/2)*R_corner, params.z)
        calculated_angles += computeIK(math.cos(step_side/2)*R_side - l_side, math.sin(step_side/2)*R_side, params.z)
        res += [calculated_angles]
    res += [toIniPos(params)]

    return res


def walkDistanceAngle(dist, angle, step_dist, step_height, params):
    """
    Retourne un tableau contenant une successions des positions clefs des 18 angles 
    des steppers permettant la marche sur la distance dist avec un angle donné
    """
    res = [toIniPos(params)]
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
    z = [-0.0, 0.11, -2, -0.0]
    
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







# def oldWalkDistanceAngle(dist, angle, step_dist, step_height, params):
#     """
#     Retourne un tableau contenant une successions des positions clefs des 18 angles 
#     des steppers permettant la marche sur la distance dist avec un angle donné
#     """
#     res = []
#     res += [toIniPos(params)]

#     nb_step = int(dist//step_dist)

#     if(nb_step != 0):
#         # first half-step
#         # calculated_angles = []
#         # for i in range(3):
#         #     calculated_angles += computeIKOriented(step_dist/4, 0, step_height, i*2+1, params, extra_angle=angle)
#         #     calculated_angles += computeIKOriented(-step_dist/4, 0, 0, i*2+2, params, extra_angle=angle)
#         # res += [calculated_angles]
#         # calculated_angles = []
#         # for i in range(3):
#         #     calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
#         #     calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
#         # res += [calculated_angles]

#         # steps (nb_steps-1)
#         for j in range(nb_step-1):
#             calculated_angles = []
#             for i in range(3):
#                 calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
#                 calculated_angles += computeIKOriented(0, 0, step_height, i*2+2, params, extra_angle=angle)
#             res += [calculated_angles]
#             calculated_angles = []
#             for i in range(3):
#                 calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
#                 calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
#             res += [calculated_angles]
#             calculated_angles = []
#             for i in range(3):
#                 calculated_angles += computeIKOriented(0, 0, step_height, i*2+1, params, extra_angle=angle)
#                 calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
#             res += [calculated_angles]
#             calculated_angles = []
#             for i in range(3):
#                 calculated_angles += computeIKOriented(step_dist/2, 0, 0, i*2+1, params, extra_angle=angle)
#                 calculated_angles += computeIKOriented(-step_dist/2, 0, 0, i*2+2, params, extra_angle=angle)
#             res += [calculated_angles]

#         last half-step
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(step_dist/4, 0, 0, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(-step_dist/4, 0, step_height, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]

#     reste 
#     if (reste > eps):
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(reste/4, 0, step_height, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(-reste/4, 0, 0, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(reste/2, 0, 0, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(-reste/2, 0, 0, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(reste/4, 0, 0, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(-reste/4, 0, step_height, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]
#         calculated_angles = []
#         for i in range(3):
#             calculated_angles += computeIKOriented(0, 0, 0, i*2+1, params, extra_angle=angle)
#             calculated_angles += computeIKOriented(0, 0, 0, i*2+2, params, extra_angle=angle)
#         res += [calculated_angles]
    
#    return res