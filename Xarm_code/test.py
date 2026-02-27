#!/usr/bin/env python3

import time
from xarm.wrapper import XArmAPI
from morradi import move


arm = XArmAPI('192.168.1.225')
time.sleep(0.5)
arm.set_tcp_maxacc(1000)
speed = 600
move = move(speed=speed, arm=arm)
arm.set_mode(0)
arm.set_state(0)
move.home()


#move to pick the object
move.jork()
time.sleep(1)
move.home()
arm.disconnect()
