#! /usr/bin/python

from quadcopter import Quadcopter
import time

copter = Quadcopter('/dev/ttyS0', 57600)

def test_1():
    copter.arm()
    time.sleep(1)
    copter.disarm()


def test_2():
    copter.takeoff(3)
    copter.setMode("LAND")


def test_3():
    copter.takeoff(5)
    copter.setOffsetVelocity(1,0,0)
    time.sleep(2)
    copter.setOffsetVelocity(0,1,0)
    time.sleep(2)
    copter.setOffsetVelocity(0,0,1)
    time.sleep(2)


if __name__=="__main__":
    test_1()
