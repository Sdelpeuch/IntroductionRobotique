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
        z=-0.06,
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

def say_hello(legs_id, params):
    t = time.time()
    if leg_id == 3 or leg_id == 6:
        return 0.05 * math.cos(t), 0.05 * math.cos(t), 0.1+ 0.5* math.fabs(math.cos(t))
    else :
        return 0.01 * math.cos(t), 0.01 * math.cos(t), 0


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
        T.append(kinematics.rotaton_2D(pt[0], pt[1] , pt[2], leg_angle))
        T[-1][0] += leg_center_pos[0] # Ajout l'offset de l'épaule
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
        controls[3], [x, y, z+0.1], p.getQuaternionFromEuler([0, 0, 0]))
    return x, y, z

last_angles = []

def from_list_to_simu(list_of_angles):
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
elif args.mode == "inverse":
    crosses = []
    for i in range(5):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True) # [0, constants.THETA2_MOTOR_SIGN * constants.theta2Correction, constants.THETA3_MOTOR_SIGN * constants.theta3Correction]
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "mouse":
    keys_z = -0.01

    # test mapping pad
    # mouse.verboseMapping(1, 0, 10, 0, 100)
    # mouse.verboseMapping(5, 0, 10, -50, 50)

elif args.mode == "walk":
    last_angles = 18 * [0]

elif args.mode == "walk-configurable":
    last_angles = 18 * [0]
    controls["angle"] = p.addUserDebugParameter("angle", 0, 360, 0)
    controls["speed"] = p.addUserDebugParameter("speed (%)", 0, 1, 0)


dt = 1/100000

while True and "walk" not in args.mode:
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

        # points = LEG_CENTER_POS # --> met une croix sur les 6 épaules (mettre leg_angle à 0)
        # i = -1
        # T = []
        # print("#####")
        # for pt in points:
        #     # Drawing each step of the DK calculation
        #     i += 1
        #     T.append(kinematics.rotaton_2D(pt[0], pt[1] , pt[2], leg_angle))
        #     T[-1][0] += leg_center_pos[0] # Ajout l'offset de l'épaule
        #     T[-1][1] += leg_center_pos[1]
        #     T[-1][2] += leg_center_pos[2]
        #     print("Drawing cross {} at {}".format(i, T[-1]))
        #     p.resetBasePositionAndOrientation(
        #         crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
        #     )
        print_points(points)

        sim.setRobotPose(
            [0, 0, 0.5],
            to_pybullet_quaternion(0, 0, 0),
        )
        state = sim.setJoints(targets)

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
            crosses [-1], T, to_pybullet_quaternion(0, 0, leg_angle)
        )
    elif args.mode == "robot-ik":
        None
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

    elif args.mode == "mouse":
        # mouse
        mp = mouse.getMousePosition()
        mouse_x, mouse_y = mouse.mappingPad(mp[0], 0, 768, -0.1, 0.1), mouse.mappingPad(mp[1], 0, 1366, -0.1, 0.1)
        # Use your own IK function
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                mouse_x,
                mouse_y,
                keys_z,
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
    elif args.mode == "walk-configurable":
        angle = p.readUserDebugParameter(controls["angle"])
        sample = kinematics.walkDistanceAngle(1, angle, 0.15, 0.1, params)
        from_list_to_simu(sample)

    elif args.mode == "hello":
        for leg_id in range(1, 7):
            x, y, z = say_hello(leg_id,params)
            alphas = kinematics.computeIKOriented(
                x,
                y,
                z,
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
    sample = kinematics.walkDistanceAngle(1, 0, 0.15, 0.1, params)
    print("time to compute :", time.time() - t)
    # print("sample : ", sample)
    t = time.time()
    from_list_to_simu(sample)
    # sample = kinematics.walkDistanceAngle(1, math.pi/2, 0.15, 0.1, params)
    # from_list_to_simu(sample)

    print("time to compute all:", time.time() - t)

elif args.mode == "walk-configurable":
    def speed_to_params(speed):
        # speed en %
        min_step_dist = 0.02
        max_step_dist = 0.1
        step_dist = max_step_dist * speed
        if step_dist < min_step_dist:
            step_dist = min_step_dist
        return step_dist
    tick = 1
    targets = {}
    while(1):
        print("im in while")
        angle = (math.pi/180)*p.readUserDebugParameter(controls["angle"])
        step_dist = speed_to_params(p.readUserDebugParameter(controls["speed"]))
        sample = kinematics.walkDistanceAngle(step_dist*2, angle, step_dist, 0.1, params)
        from_list_to_simu(sample)