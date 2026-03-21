
from time import time

import xarm.wrapper as arm

class move():
    """
   Standarised movements
    """
    
    def __init__(self, arm, speed):
        self.arm = arm
        self.speed = speed
       

    # Instance method
    def home(self):
        """
        Goes home.
        """
        self.arm.set_position(*[130, 0, 150, 180, 0, 0], speed=self.speed, wait=True)
        
    def place(self, x, z):
        """
        Places the object in the bin.
        """
        self.arm.set_position(*[x, 220.6, z, 180, 0, 0], speed=self.speed, wait=True)
        self.arm.set_gripper_position(800, wait=True)

