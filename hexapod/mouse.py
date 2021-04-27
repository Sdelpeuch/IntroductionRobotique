from pynput.mouse import Button, Controller
from pynput.keyboard import Key, Listener
import kinematics

c = Controller()

lastPos = None
lastZ = None

def didItMoved(pos, float_z):
	global lastPos, lastZ
	if lastPos == None or lastZ == None:
		lastPos = pos
		lastZ = float_z
		print("First position")
		return True
	if lastPos[0] == pos[0] and lastPos[1] == pos[1] and float_z == lastZ:
		return False
	lastPos = pos
	lastZ = float_z
	return True

def getMousePosition():
    return c.position

def mappingPad(value, v_min, v_max, new_min, new_max):
    """
    Given a value x defined in [v_min, v_max], it returns
    the value as if it was defined in [new_min, new_max]
    """
    value = value - v_min
    value = value * (new_max - new_min) / (v_max - v_min)
    value = value + new_min
    return value

def mouseRobot(params, coef=10, verbose=False, z=-5):
    mp = getMousePosition()
    float_z = float(z)
    hasMoved = didItMoved(mp, float_z)
    mouse_x, mouse_y = mappingPad(mp[0], 0, 768, -coef, coef), mappingPad(mp[1], 0, 1366, -coef, coef)
    if verbose and hasMoved: print("Mouse in pos ({}, {}) gives: [{:.2f}, {:.2f}] (z={:.2f})".format(mp[0], mp[1], mouse_x, mouse_y, float_z))
    # Use your own IK function
    alphas_list = []
    for leg_id in range(1, 7):
        alphas = kinematics.computeIKOriented(
            mouse_x,
            mouse_y,
            float_z,
            leg_id,
            params,
            verbose=False,
        )
        alphas_list.append(alphas)
    if verbose and hasMoved:
    	print(alphas_list)
    return alphas_list, hasMoved

# def on_press(key):
#     print('{0} pressed'.format(
#         key))

# def on_release(key):
#     print('{0} release'.format(
#         key))
#     if key == Key.esc:
#         # Stop listener
#         return False

# # Collect events until released
# with Listener(on_press=on_press, on_release=on_release) as listener:
#     listener.join()