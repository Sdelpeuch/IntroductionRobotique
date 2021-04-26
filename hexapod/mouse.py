from pynput.mouse import Button, Controller
from pynput.keyboard import Key, Listener

c = Controller()

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


def verboseMapping(v, vmin, vmax, nmin, nmax):
    print("{} in [{}, {}] to [{}, {}] equals {}".format(v, vmin, vmax, nmin, nmax, mouse.mappingPad(v, vmin, vmax, nmin, nmax)))


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