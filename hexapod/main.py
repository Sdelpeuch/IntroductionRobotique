import pypot.dynamixel
import time
import math
from utils import *
import time

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

def main():
    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    robot = SimpleRobot(dxl_io)

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
    def moveFromAlphas(alphas, hasMoved, verboseMain=False):
        for key, value in robot.legs.items():
            value[0].goal_position = np.degrees(alphas[key-1][0])
            value[1].goal_position = np.degrees(alphas[key-1][1])
            value[2].goal_position = np.degrees(alphas[key-1][2])
            if verboseMain and hasMoved:
                print("Leg {} recieved angles T1={}, T2={}, T3={}".format(key, value[0].goal_position, value[1].goal_position, value[2].goal_position))
            
        if hasMoved:
           # print("It moved!")
            robot.smooth_tick_read_and_write(0.5, verbose=False)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:

        params = Parameters(
            freq=100,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        for k, v in robot.legs.items():
            v[0].goal_position = read_init(init, int(10 * k + 1))
            v[1].goal_position = read_init(init, int(10 * k + 2))
            v[2].goal_position = read_init(init, int(10 * k + 3))
        robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")

        verboseMain = True
        while True:
            alphas, hasMoved = mouse.mouseRobot(params=params, coef=20, verbose=True, z = 10) #input("z?"))

            moveFromAlphas(alphas, hasMoved, verboseMain=False)
            
        # TODO create this function instead:
        # setPositionToRobot(0, 0, 0, robot, params)
        time.sleep(2)
        print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
