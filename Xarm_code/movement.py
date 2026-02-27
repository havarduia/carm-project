
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
        
    def pick_prepare(self):
        """
        Prepares to pick the object.
        """
        self.arm.set_position(*[400, 0, 300, 100, 0, 0], speed=self.speed, wait=True)
        
    def jork(self):
        """
        Jorks the arm.
        """
        
        for i in range(3):
            self.arm.set_gripper_position(800, wait=False)
            self.arm.set_position(*[550, 0, 100, 180, 0, 0], speed=self.speed, wait=False)
            self.arm.set_gripper_position(0, wait=False)
            self.arm.set_position(*[300, 0, 100, 180, 0, 0], speed=self.speed, wait=False)
        
            

