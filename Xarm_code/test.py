#!/usr/bin/env python3

import time
from xarm.wrapper import XArmAPI
from movement import move


arm = XArmAPI('192.168.1.225')
time.sleep(0.5)
arm.set_tcp_maxacc(1000)
speed = 50
move = move(speed=speed, arm=arm)
arm.set_mode(0)
arm.set_state(0)
offset_x = 10
offset_z = -20
x, y, z, px, py, pz = [269.838, 6.227, 39.363, 180, 0, 0]

#move to pick the object
move.home()
arm.set_position(*[x+offset_x, y, z+offset_z, px, py, pz], wait=True)
arm.set_gripper_position(350, wait=True)
move.home()
arm.disconnect()
