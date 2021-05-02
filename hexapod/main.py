import argparse

import pypot.dynamixel
import time
import math
from utils import *
import time
import kinematics
# from kinematics import *
import math
import mouse
import sys
from signal import signal, SIGINT
import traceback
import os
import numpy as np
# import display

init = {11:  0, 12:-20, 13:-20,
        21:-30, 22:-20, 23:-20,
        31: 30, 32:-20, 33:-20,
        41:  0, 42:-20, 43:-20,
        51:-30, 52:-20, 53:-20,
        61: 30, 62:-20, 63:-20}

def read_init(init, value):
    print("Value of leg {} is {}.".format(value, init[value]))
    return init[value]

def moveFromAlphas(robot, alphas, hasMoved, verboseMain=False):
    for key, value in robot.legs.items():
        value[0].goal_position = alphas[key-1][0]
        value[1].goal_position = alphas[key-1][1]
        value[2].goal_position = alphas[key-1][2]
        if verboseMain and hasMoved:
            print("Leg {} recieved angles T1={}, T2={}, T3={}".format(key, value[0].goal_position, value[1].goal_position, value[2].goal_position))

    if hasMoved:
       # print("It moved!")
        robot.smooth_tick_read_and_write(0.5, verbose=False)

def main():
    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    robot = SimpleRobot(dxl_io)
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
    args = parser.parse_args()
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:
        print("Setting initial position")
        for k, v in robot.legs.items():
            v[0].goal_position = read_init(init, int(10 * k + 1))
            v[1].goal_position = read_init(init, int(10 * k + 2))
            v[2].goal_position = read_init(init, int(10 * k + 3))
        robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")

        if args.mode == "mouse":
            verboseMain = False
            while True:
                alphas, hasMoved = mouse.mouseRobot(params=robot.params, coef=20, verbose=True, z = 0.01) #input("z?"))
                moveFromAlphas(robot, alphas, hasMoved, verboseMain=False)
            time.sleep(2)
            print("Closing")

        elif args.mode == "sinus":
            pi = math.pi
            i = 0
            while True:
                for k,v in robot.legs.items():
                    theta1, theta2, theta3 = kinematics.computeIKOriented(0,0 , i, 1, robot.params)
                    v[0].goal_position = theta1
                    v[1].goal_position = theta2
                    v[2].goal_position = theta3
                    print(theta1, theta2, theta3)
                    i += 1
                robot.smooth_tick_read_and_write(3, verbose=False)

        else :
            v = robot.legs[1]
            theta1, theta2, theta3 = kinematics.computeIKOriented(0.35,0.17,0.01,1,robot.params)
            v[0].goal_position = theta1
            v[1].goal_position = theta2
            v[2].goal_position = theta3
            print(theta1,theta2,theta3)
            robot.smooth_tick_read_and_write(3, verbose=False)
            theta1, theta2, theta3 = kinematics.computeIKOriented(100, 500, 0.01, 1, robot.params)
            v[0].goal_position = theta1
            v[1].goal_position = theta2
            v[2].goal_position = theta3
            print(theta1, theta2, theta3)
            robot.smooth_tick_read_and_write(3, verbose=False)
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
