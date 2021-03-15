import os
import argparse
import math
import time
import pybullet as p
if __package__ is None or __package__ == '':
    import control
else:
    from . import control

t = 0.
dt = 0.01
dirname = os.path.dirname(__file__) + '/models/'


def init():
    """Initialise le simulateur"""
    # Instanciation de Bullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Chargement du sol
    planeId = p.loadURDF(dirname+'/plane.urdf')

    p.setPhysicsEngineParameter(fixedTimeStep=dt)


def loadModel(name, fixed=False, startPos=[0., 0., 0.1], startOrientation=[0., 0., 0.]):
    """Charge un modèle"""
    startOrientation = p.getQuaternionFromEuler(startOrientation)
    model = p.loadURDF(dirname+"/"+name+"/robot.urdf",
                       startPos, startOrientation, useFixedBase=fixed)

    return model


def setJoints(robot, joints):
    """Définis les angles cibles pour les moteurs du robot

    Arguments:
        int -- identifiant du robot
        joints {list} -- liste des positions cibles (rad)
    """
    jointsMap = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(
            robot, jointInfo[0], p.POSITION_CONTROL, joints[k])


def inverseControls(name='', x_default=0.15, y_default=0.):
    target_x = p.addUserDebugParameter(
        '%starget_x' % name, -0.3, 0.3, x_default)
    target_y = p.addUserDebugParameter(
        '%starget_y' % name, -0.3, 0.3, y_default)
    target_z = p.addUserDebugParameter('%starget_z' % name, -0.3, 0.3, 0.)
    target = loadModel('target2', True)

    return target_x, target_y, target_z, target


def inverseUpdate(controls):
    x = p.readUserDebugParameter(controls[0])
    y = p.readUserDebugParameter(controls[1])
    z = p.readUserDebugParameter(controls[2])
    p.resetBasePositionAndOrientation(
        controls[3], [x, y, z+0.1], p.getQuaternionFromEuler([0, 0, 0]))

    return x, y, z

def tick():
    global t

    t += dt
    p.stepSimulation()
    time.sleep(dt)

if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser(prog="Quadruped")
    parser.add_argument('-m', type=str, help='Mode', default='motors')
    args = parser.parse_args()
    mode = args.m

    if mode not in ['motors', 'sandbox', 'direct', 'inverse', 'draw', 'legs', 'walk']:
        print('Le mode %s est inconnu' % mode)
        exit(1)

    init()
    fixed = False
    startPos = [0., 0., 0.1]
    startOrientation = [0., 0., 0.]

    if mode == 'motors':
        motors_sliders = []
        for k in range(12):
            motors_sliders.append(p.addUserDebugParameter(
                "motor_%d" % k, -3.14, 3.14, 0.))
    elif mode == 'direct':
        fixed = True
        startOrientation = [0., 0., math.pi + math.pi/4]
        startPos = [-0.04, 0., 0.1]
        motor_1 = p.addUserDebugParameter('motor_1', -1, 1, 0)
        motor_2 = p.addUserDebugParameter('motor_1', -1, 1, 0)
        motor_3 = p.addUserDebugParameter('motor_1', -1, 1, 0)
        target = loadModel('target2', True)
    elif mode == 'inverse':
        fixed = True
        startOrientation = [0., 0., math.pi + math.pi/4]
        startPos = [-0.04, 0., 0.1]
        leg = inverseControls()
    elif mode == 'draw':
        fixed = True
        startOrientation = [0., 0., math.pi + math.pi/4]
        startPos = [-0.04, 0., 0.1]
        lastLine = None
    elif mode == 'legs':
        fixed = True
        leg1 = inverseControls('leg1_', -0.15, 0.15)
        leg2 = inverseControls('leg2_', -0.15, -0.15)
        leg3 = inverseControls('leg3_', 0.15, -0.15)
        leg4 = inverseControls('leg4_', 0.15, 0.15)

    elif mode == 'walk':
        speed_x = p.addUserDebugParameter('speed_x', -0.2, 0.2, 0.)
        speed_y = p.addUserDebugParameter('speed_y', -0.2, 0.2, 0.)
        speed_rotation = p.addUserDebugParameter(
            'speed_rotation', -0.5, 0.5, 0.)
    elif mode == 'sandbox':
        print('Mode bac à sable...')

    robot = loadModel('quadruped', fixed, startPos, startOrientation)

    # Boucle principale
    while True:
        if mode == 'motors':
            joints = []
            for entry in motors_sliders:
                joints.append(p.readUserDebugParameter(entry))
        elif mode == 'inverse':
            joints = control.inverse(*inverseUpdate(leg)) + [0]*9
        elif mode == 'direct':
            a = p.readUserDebugParameter(motor_1)
            b = p.readUserDebugParameter(motor_2)
            c = p.readUserDebugParameter(motor_3)
            joints = [a,b,c] + [0]*9

            x, y, z = control.direct(a, b, c)

            p.resetBasePositionAndOrientation(target, [x, y, z+0.1], p.getQuaternionFromEuler([0, 0, 0]))
        elif mode == 'draw':
            joints = control.draw(t) + [0]*9

            def getLegTip():
                res = p.getLinkState(robot, 3)
                return res[0]

            if lastLine is None:
                lastLine = time.time(), getLegTip()
            elif time.time() - lastLine[0] > 0.1:
                tip = getLegTip()
                p.addUserDebugLine(lastLine[1], tip, (1., 0, 0), 2., 10.)
                lastLine = time.time(), tip
        elif mode == 'legs':
            leg1_xyz = inverseUpdate(leg1)
            leg2_xyz = inverseUpdate(leg2)
            leg3_xyz = inverseUpdate(leg3)
            leg4_xyz = inverseUpdate(leg4)

            joints = control.legs(leg1_xyz, leg2_xyz, leg3_xyz, leg4_xyz)
        elif mode == 'walk':
            x = p.readUserDebugParameter(speed_x)
            y = p.readUserDebugParameter(speed_y)
            rotation = p.readUserDebugParameter(speed_rotation)
            joints = control.walk(t, x, y, rotation)
        elif mode == 'sandbox':
            joints = control.sandbox(t)

        # Envoi des positions cibles au simulateur
        setJoints(robot, joints)

        # Mise à jour de la simulation
        tick()
