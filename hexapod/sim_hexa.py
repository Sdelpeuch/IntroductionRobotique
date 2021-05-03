#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation

import mouse

import kinematics
from constants import *

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


class Parameters:
    def __init__(
        self,
        z=-0.1,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]
    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


def attack(params):
    """
    Idle position.
    Its body moves with a little sinus and its front legs wave as if it was attacking someone
    """
    angles = []
    t = time.time()
    for leg_id in range(1, 7):

        if leg_id == 1 or leg_id == 2:
            angles.append([0.1 * math.fabs(math.tan(t)), 0, 0.1 + 0.2 * math.fabs(math.cos(t))])
        elif leg_id == 3 or leg_id == 6:
            angles.append([0.1, 0.01 * math.cos(t), 0.01 * math.cos(t)])
        else:
            angles.append([0.01 * math.cos(t), 0.01 * math.cos(t), 0.01])

    for leg_id in range(1, 7):
        angle = angles[leg_id - 1]
        alphas = kinematics.computeIKOriented(
            angle[0],
            angle[1],
            angle[2],
            leg_id,
            params,
            verbose=False,
        )
        set_leg_angles(alphas, leg_id, targets, params)
    state = sim.setJoints(targets)
    sim.tick()


def holowalk(x_speed, y_speed, behaviour, params):
    if x_speed == 0 and y_speed == 0 and th_speed == 0 and behaviour != "JUMP" and behaviour != "ROTATE_R" and behaviour != "ROTATE_L":
        return "ATTACK", attack(params)
    elif x_speed != 0 or y_speed != 0:
        if x_speed > 0 and y_speed > 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, math.pi / 4, 0.15, 0.1, params)
        if x_speed < 0 and y_speed > 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, 3 * math.pi / 4, 0.15, 0.1, params)
        if x_speed < 0 and y_speed < 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, 5 * math.pi / 4, 0.15, 0.1, params)
        if x_speed > 0 and y_speed < 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, 7 * math.pi / 4, 0.15, 0.1, params)

        if x_speed > 0 and y_speed == 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, 0, 0.15, 0.1, params)
        if x_speed < 0 and y_speed == 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, math.pi, 0.15, 0.1, params)
        if x_speed == 0 and y_speed > 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, math.pi / 2, 0.15, 0.1, params)
        if x_speed == 0 and y_speed < 0:
            return "WALK", kinematics.walkDistanceAngle(0.3, 3 * math.pi / 2, 0.15, 0.1, params)
    elif behaviour == "JUMP":
        return "JUMP", kinematics.jump(params=params)
    elif behaviour == "ROTATE_R":
        return "ROTATE_R", kinematics.rotate(0.15*2, 0.15, 0.2, params)
    elif behaviour == "ROTATE_L":
        return "ROTATE_L", kinematics.rotate(-0.15*2, 0.15, 0.2, params)

# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters()


def print_points(points):
    i = -1
    T = []
    print("#####")
    for pt in points:
        # Drawing each step of the DK calculation
        i += 1
        T.append(kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle))
        T[-1][0] += leg_center_pos[0]  # Ajout l'offset de l'épaule
        T[-1][1] += leg_center_pos[1]
        T[-1][2] += leg_center_pos[2]
        print("Drawing cross {} at {}".format(i, T[-1]))
        p.resetBasePositionAndOrientation(
            crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
        )


def inverseUpdate(controls):
    x = p.readUserDebugParameter(controls[0])
    y = p.readUserDebugParameter(controls[1])
    z = p.readUserDebugParameter(controls[2])
    p.resetBasePositionAndOrientation(
        controls[3], [x, y, z + 0.1], p.getQuaternionFromEuler([0, 0, 0]))
    return x, y, z


last_angles = []


def from_list_to_simu(list_of_angles, dt=1 / 1000):
    global last_angles
    for step in list_of_angles:
        smooth_steps = kinematics.make_smooth(step, last_angles, smooth_num=10)
        last_angles = step
        # smooth_steps = [step]
        for smooth_step in smooth_steps:
            for leg_id in range(1, 7):
                index = (leg_id - 1) * 3
                alphas = [smooth_step[index],
                          smooth_step[index + 1],
                          smooth_step[index + 2]]
                set_leg_angles(alphas, leg_id, targets, params)
                state = sim.setJoints(targets)
                sim.tick()
                time.sleep(dt)


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)

        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

elif args.mode == "robot-ik-keyboard":
    x_body, y_body, z_body = 0, 0, params.z
    max_value = 0.05
    value = 0.001

elif args.mode == "inverse":
    crosses = []
    for i in range(5):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0,
                                  use_rads=True)  # [0, constants.THETA2_MOTOR_SIGN * constants.theta2Correction, constants.THETA3_MOTOR_SIGN * constants.theta3Correction]
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "walk" or args.mode == "rotate":
    last_angles = 18 * [0]

elif args.mode == "walk-configurable":
    last_angles = 18 * [0]
    controls["angle"] = p.addUserDebugParameter("angle", 0, 360, 0)
    controls["step_dist"] = p.addUserDebugParameter("step distance", 0, 0.3, 0)
    controls["freq"] = p.addUserDebugParameter("frequence", 100, 10000, 5000)

elif args.mode == "holo":
    last_angles = 18 * [0]
    behaviour = "ATTACK"
elif args.mode == "walk-jump":
    last_angles = 18 * [0]

dt = 1 / 10000

while True and "walk" not in args.mode and "holo" not in args.mode and "rotate" not in args.mode:
    tick = 1
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0

    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        # Use your own DK function, the result should be: https://www.youtube.com/watch?v=w3psAbh3AoM
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )

        print_points(points)

        sim.setRobotPose(
            [0, 0, 0.5],
            to_pybullet_quaternion(0, 0, 0),
        )
        state = sim.setJoints(targets)

    elif args.mode == "keyboard":
        # Affiche le code de la touche appuyée
        # Mode simplement utilise pendant le développement
        keys = p.getKeyboardEvents()
        print(keys)

    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
            print("name:", name, ", valeur:", targets[name])
        state = sim.setJoints(targets)

    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        points = kinematics.computeDKDetailed(alphas[0], alphas[1], alphas[2])
        print_points(points)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            crosses[-1], T, to_pybullet_quaternion(0, 0, leg_angle)
        )

    elif args.mode == "robot-ik":
        # Use your own IK function
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                leg_id,
                params,
                verbose=False,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        # sim.setRobotPose(
        #     [0, 0, 0.5],
        #     to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)

    elif args.mode == "robot-ik-keyboard":
        keys = p.getKeyboardEvents()
        if 122 in keys:
            x_body = min(x_body + value, max_value)
        if 115 in keys:
            x_body = max(x_body - value, - max_value)

        if 113 in keys:
            y_body = min(y_body + value, max_value)
        if 100 in keys:
            y_body = max(y_body - value, - max_value)

        if 101 in keys:
            z_body = min(z_body + value, params.z + max_value)
        if 97 in keys:
            z_body = max(z_body - value, params.z - max_value)

        # print("{}, {}, {}".format(x_body, y_body, z_body))

        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                x_body,
                y_body,
                z_body,
                leg_id,
                params,
                verbose=False,
            )

            set_leg_angles(alphas, leg_id, targets, params)
        # sim.setRobotPose(
        #     [0, 0, 0.5],
        #     to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)

    elif args.mode == "attack":
        angles = attack(params)
        for leg_id in range(1, 7):
            angle = angles[leg_id - 1]
            alphas = kinematics.computeIKOriented(
                angle[0],
                angle[1],
                angle[2],
                leg_id,
                params,
                verbose=False,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

    sim.tick()


if args.mode == "walk":
    tick = 1
    targets = {}
    t = time.time()
    sample = kinematics.walkDistanceAngle(1.5, 0, 0.15, 0.1, params)
    print("time to compute :", time.time() - t)
    # print("sample : ", sample)
    t = time.time()
    print("time to compute all:", time.time() - t)
    from_list_to_simu(sample)
    # sample = kinematics.walkDistanceAngle(1, math.pi/2, 0.15, 0.1, params)
    # from_list_to_simu(sample)

elif args.mode == "rotate":
    tick = 1
    targets = {}
    t = time.time()
    sample = kinematics.rotate(2*math.pi, 0.15, 0.1, params)
    print("time to compute :", time.time() - t)
    #print("sample : ", sample)
    t = time.time()
    print("time to compute all:", time.time() - t)
    from_list_to_simu(sample)

elif args.mode == "walk-configurable":
    tick = 1
    targets = {}
    while (1):
        angle = (math.pi / 180) * p.readUserDebugParameter(controls["angle"])
        step_dist = p.readUserDebugParameter(controls["step_dist"])
        dt_factor = p.readUserDebugParameter(controls["freq"])
        if step_dist != 0:
            sample = kinematics.walkDistanceAngle(step_dist * 2, angle, step_dist, 0.1, params)
            from_list_to_simu(sample, 1 / dt_factor)
        else:
            attack(params)

elif args.mode == "holo":
    tick = 1
    targets = {}
    t = time.time()
    while True:
        keys = p.getKeyboardEvents()
        x, y, th_speed = 0, 0, 0
        if behaviour == "ATTACK":
            if 122 in keys:
                x = 10
            elif 115 in keys:
                x = -10

            if 113 in keys:
                y = -10
            elif 100 in keys:
                y = 10

            if 101 in keys:
                behaviour = "ROTATE_R"
            elif 97 in keys:
                behaviour = "ROTATE_L"

            if 32 in keys and keys[32] == 4:
                behaviour = "JUMP"

        behaviour, angles = holowalk(x, y, behaviour, params)
        if behaviour == "ATTACK":
            attack(params)
        elif behaviour == "WALK":
            from_list_to_simu(angles)
        elif behaviour == "JUMP":
            from_list_to_simu(angles)
            behaviour = "ATTACK"
        elif behaviour == "ROTATE_R" or behaviour == "ROTATE_L":
            from_list_to_simu(angles)
            behaviour = "ATTACK"


elif args.mode == "walk-jump":
    tick = 1
    targets = {}
    while (1):
        keys = p.getKeyboardEvents()
        # On trigger le jump que quand on release la barre espace
        # Pour trigger quand on appuie dessus, mettre keys[32] == 3
        if 32 in keys and keys[32] == 4:
            sample = kinematics.jump(params=params)
            from_list_to_simu(sample)
