import inputs
import time
from multiprocessing import Process
from multiprocessing.sharedctypes import Value
from ctypes import c_double, c_bool, c_int
from pynput import keyboard
from setuptools._vendor.more_itertools.more import combination_index
import Types

class KeyboardClient():
    def __init__(self):
        self.running = Value(c_bool, lock=True)
        
        self.speedX = Value(c_double, lock=True)
        self.speedY = Value(c_double, lock=True)
        self.speedZ = Value(c_double, lock=True)
        
        self.bodyX = Value(c_double, lock=True)
        self.bodyY = Value(c_double, lock=True)
        self.bodyZ = Value(c_double, lock=True)
        
        self.bodyAngleX = Value(c_double, lock=True)
        self.bodyAngleY = Value(c_double, lock=True)
        self.bodyAngleZ = Value(c_double, lock=True)
        
        self.gaitCode = Value(c_int, lock=True)

        self.speedX.value = 0.0
        self.speedY.value = 0.0
        self.speedZ.value = 0.0

        self.bodyX.value = 0.0
        self.bodyY.value = 0.0
        self.bodyZ.value = 0.0
        self.running.value = True
        self.gaitCode.value = 0 

        listener = keyboard.Listener(on_press=self.onKeyPressed,on_release=self.onKeyRelease)
        listener.start()
        time.sleep(0.1)
        
        self.currentlyPressed = set()
        self.combination = {keyboard.Key.ctrl, keyboard.Key.shift}
        self.gaitChanged = False
        self.speedChanged = False
        self.bodyChanged = False
        self.changed = False

    # returns (bool, changedGait), but only once, subsequent calls will return (False, 0) 
    def gaitHasChanged(self):
        code = self.gaitCode.value
        changed = self.gaitChanged
        self.gaitChanged = False
        self.gaitCode.value = 0
        return (changed, code)
    
    # returns true until ESC has been called
    def stillRunning(self):
        return self.running.value
        
    # Listener to keyboard, is called whenever a key is pressed
    def onKeyPressed(self, key):
        try:

            if key in self.combination:
                #print ("add",key)
                self.currentlyPressed.add(key)
            else:                    
                #print('Alphanumeric key pressed: {0} '.format(key))
                if key.char == '1':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.Pacing.value
                if key.char == '2':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.Bounding.value
                if key.char == '3':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.Walking.value
                if key.char == '4':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.Trot.value
                if key.char == '5':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.WalkingTrot.value
                if key.char == '6':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.CustomGallop.value
                if key.char == '7':
                    self.gaitChanged = True
                    self.gaitCode.value = Types.GaitType.NoMovement.value

        except AttributeError:
            doBodyMove = keyboard.Key.shift in self.currentlyPressed
            doBodyRotate = keyboard.Key.ctrl in self.currentlyPressed
            #print('special key pressed: {0}'.format(key), " mode:", 'BodyMove' if doBodyMove else 'BodyRotate' if doBodyRotate else 'speed')
            if key == keyboard.Key.esc:
               self.changed = True
               self.running.value = False

            if not doBodyMove and not doBodyRotate:
                if key == keyboard.Key.up :
                    self.speedChanged = True
                    self.speedX.value = self.speedX.value + 0.1
                if key == keyboard.Key.down:
                    self.speedChanged = True
                    self.speedX.value = self.speedX.value - 0.1
                if key == keyboard.Key.page_up :
                    self.speedChanged = True
                    self.speedZ.value = self.speedZ.value + 0.1
                if key == keyboard.Key.page_down:
                    self.speedChanged = True
                    self.speedZ.value = self.speedZ.value - 0.1
                if key == keyboard.Key.left:
                    self.speedChanged = True
                    self.speedY.value = self.speedY.value + 0.1
                if key == keyboard.Key.right:
                    self.speedChanged = True
                    self.speedY.value = self.speedY.value - 0.1

            if doBodyMove:
                if key == keyboard.Key.up :
                    self.changed = True
                    self.bodyX.value = self.bodyX.value + 0.1

                if key == keyboard.Key.down:
                    self.bodyChanged = True
                    self.bodyX.value = self.bodyX.value - 0.1
                if key == keyboard.Key.page_up :
                    self.bodyChanged = True
                    self.bodyZ.value = self.bodyZ.value + 0.1
                if key == keyboard.Key.page_down:
                    self.bodyChanged = True
                    self.bodyZ.value = self.bodyZ.value - 0.1
                if key == keyboard.Key.left:
                    self.bodyChanged = True
                    self.bodyY.value = self.bodyY.value + 0.1
                if key == keyboard.Key.right:
                    self.bodyChanged = True
                    self.bodyY.value = self.bodyY.value - 0.1
                    
            if doBodyRotate:
                if key == keyboard.Key.up :
                    self.bodyChanged = True
                    self.bodyAngleX.value = self.bodyAngleX.value + 0.1
                if key == keyboard.Key.down:
                    self.bodyChanged = True
                    self.bodyAngleX.value = self.bodyAngleX.value - 0.1
                if key == keyboard.Key.page_up :
                    self.bodyChanged = True
                    self.bodyAngleZ.value = self.bodyAngleZ.value + 0.1
                if key == keyboard.Key.page_down:
                    self.bodyChanged = True
                    self.bodyAngleZ.value = self.bodyAngleZ.value - 0.1
                if key == keyboard.Key.left:
                    self.bodyChanged = True
                    self.bodyAngleY.value = self.bodyAngleY.value + 0.1
                if key == keyboard.Key.right:
                    self.bodyChanged = True
                    self.bodyAngleY.value = self.bodyAngleY.value - 0.1
                
        if self.gaitChanged or self.speedChanged or self.bodyChanged:
            self.changed = True

    def onKeyRelease(self,key):
        #print('Key released: {0}'.format(key))
        try:
            self.currentlyPressed.remove(key)
            #print ("remove",key)

        except KeyError:
            pass
    

if __name__ == "__main__":
    print("start keyboard test client")
    gp = KeyboardClient()
    for i in range(1000):
        if gp.changed:
            print("speedX %3.1f " % gp.speedX.value, end=" ; ")
            print("speedY %3.1f " % gp.speedY.value, end=" ; ")
            print("speedZ %3.1f " % gp.speedZ.value)

            
            print("bodyX %3.1f " % gp.bodyX.value, end=" ; ")
            print("bodyY %3.1f " % gp.bodyY.value, end=" ; ")
            print("bodyZ %3.1f " % gp.bodyZ.value)

    
            print("bodyAngleX %3.1f " %gp.bodyAngleX.value, end=" ; ")
            print("bodyAngleY %3.1f " %gp.bodyAngleY.value, end=" ; ")
            print("bodyAngleZ %3.1f " %gp.bodyAngleZ.value)
            gp.changed = False
        else:
            time.sleep(0.1)
