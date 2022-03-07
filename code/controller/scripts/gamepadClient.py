'''
Simple python script to get Asyncronous gamepad inputs
Thomas FLAYOLS - LAAS CNRS
From https://github.com/thomasfla/solopython

Use:
To display data, run "python gamepadClient.py"
'''
import inputs
import time
from multiprocessing import Process
from multiprocessing.sharedctypes import Value
from ctypes import c_double, c_bool
from pynput import keyboard
from omniORB.CORBA import TRUE

class GamepadClient():
    def __init__(self):
        self.running = Value(c_bool, lock=True)
        self.startButton = Value(c_bool, lock=True)
        self.backButton = Value(c_bool, lock=True)
        self.leftJoystickX = Value(c_double, lock=True)
        self.leftJoystickY = Value(c_double, lock=True)
        self.rightJoystickX = Value(c_double, lock=True)
        self.rightJoystickY = Value(c_double, lock=True)
        self.R1Button = Value(c_bool, lock=True)
        self.L1Button = Value(c_bool, lock=True)

        self.gaitCode = 5 # Static
        self.startButton.value = False
        self.backButton.value = False
        self.leftJoystickX.value = 0.0
        self.leftJoystickY.value = 0.0
        self.rightJoystickX.value = 0.0
        self.rightJoystickY.value = 0.0
        self.R1Button.value = False
        self.L1Button.value = False

        args = (self.running, self.startButton, self.backButton,
                self.gaitCode, self.leftJoystickX,
                self.leftJoystickY, self.rightJoystickX, self.rightJoystickY, self.R1Button, self.L1Button)
        self.process = Process(target=self.run, args=args)
        self.process.start()

        listener = keyboard.Listener(on_press=self.onKeyPressed,on_release=self.onKeyRelease)
        listener.start()
        time.sleep(0.2)

    # Listener to keyboard, is called whenever a key is pressed
    def onKeyPressed(self, key):
        self.gaitCode = 0

        try:
            print('Alphanumeric key pressed: {0} '.format(key.char))
            if key.char == 'L':
                self.L1Button.value = True
            if key.char == 'l':
                self.L1Button.value = False
            if key.char == 'R':
                self.R1Button.value = True
            if key.char == 'r':
                self.R1Button.value = False
            if key.char == 'S':
                self.startButton.value = True
            if key.char == 's':
                self.startButton.value = False
            if key.char == '1':
                self.gaitCode = 1
            if key.char == '2':
                self.gaitCode = 2
            if key.char == '3':
                self.gaitCode = 3
            if key.char == '4':
                self.gaitCode = 4
            if key.char == '5':
                self.gaitCode = 5
            if key == keyboard.Key.ESC:
                self.backButton.value = True

        except AttributeError:
            print('special key pressed: {0}'.format(key))
            
            if key == keyboard.Key.up :
                self.leftJoystickY.value = self.leftJoystickY.value + 0.1
            if key == keyboard.Key.down:
                self.leftJoystickY.value = self.leftJoystickY.value - 0.1
            if key == keyboard.Key.page_up :
                self.leftJoystickX.value = self.leftJoystickX.value + 0.1
            if key == keyboard.Key.page_down:
                self.leftJoystickX.value = self.leftJoystickX.value - 0.1
            if key == keyboard.Key.left:
                self.rightJoystickX.value = self.rightJoystickX.value + 0.1
            if key == keyboard.Key.right:
                self.rightJoystickX.value = self.rightJoystickX.value - 0.1
            print('leftJoyStickX {0}'.format(self.leftJoystickX.value))
            
            


    def onKeyRelease(self,key):
        print('Key released: {0}'.format(key))
    
    def run(self, running, startButton, backButton, gaitCode, leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY, R1Button, L1Button):
        running.value = True

        # Set listeners for keyboard events
        #listener.start()
    
        while(running.value):

            #events = inputs.get_gamepad()
            #for event in events:
            if 1==0:
                # print(event.ev_type, event.code, event.state)
                if event.ev_type == 'Absolute':
                    if event.code == 'ABS_X':
                        leftJoystickX.value = event.state / 32768.0
                    if event.code == 'ABS_Y':
                        leftJoystickY.value = event.state / 32768.0
                    if event.code == 'ABS_RX':
                        rightJoystickX.value = event.state / 32768.0
                    if event.code == 'ABS_RY':
                        rightJoystickY.value = event.state / 32768.0
                if (event.ev_type == 'Key'):
                    if event.code == 'BTN_START':
                        startButton.value = event.state
                    elif event.code == 'BTN_TR':
                        R1Button.value = event.state
                    elif event.code == 'BTN_TL':
                        L1Button.value = event.state
                    elif event.code == 'BTN_SELECT':
                        backButton.value = event.state

    def stop(self):
        self.running.value = False
        self.process.terminate()
        self.process.join()


if __name__ == "__main__":
    gp = GamepadClient()
    for i in range(1000):
        print("LX = ", gp.leftJoystickX.value, end=" ; ")
        print("LY = ", gp.leftJoystickY.value, end=" ; ")
        print("RX = ", gp.rightJoystickX.value, end=" ; ")
        print("RY = ", gp.rightJoystickY.value, end=" ; ")
        print("start = ",gp.startButton.value)
        print("back = ",gp.backButton.value)
        print("R1 = ",gp.R1Button.value)
        print("L1 = ",gp.L1Button.value)
        time.sleep(0.1)

    gp.stop()
