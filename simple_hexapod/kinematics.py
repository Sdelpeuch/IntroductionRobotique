# not empty file
import numpy as np 
from scipy.spatial.transform import Rotation as R

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
