from pynput import keyboard
import time;

def onKeyPressed(key):
    try:
        print('Alphanumeric key pressed: {0} '.format(
            key.char))
    except AttributeError:
        print('special key pressed: {0}'.format(
            key))

def onKeyReleased(key):
    print('Key released: {0}'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Collect events until released
listener = keyboard.Listener(
        on_press=onKeyPressed,
        on_release=onKeyReleased)
listener.start()

time.sleep(100)