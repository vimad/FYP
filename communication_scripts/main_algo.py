#! /usr/bin/python

from dronekit import LocationGlobal

from communication.quadcopter import Quadcopter
from communication.util import getDistance

import os, sys, time, math, errno


tagID = 13

kp = 1.0
kd = 0.5
ki = 0

kpz = 0.01
kdz = 0.01

FIFO_R = '/tmp/fifo_c-p'
FIFO_W = '/tmp/fifo_p-c'


try:
    os.mkfifo(FIFO_R)
    os.mkfifo(FIFO_W)

except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise


def main(connectString = "/dev/ttyS0", baud = 57600):

    copter = Quadcopter(connection_string = connectString, baud = baud)

    foundTag = False
    seeked = False
   
    cmds = copter.downloadMission()
    landing = cmds[-1]

    for cmd in cmds:
        if cmd.command == 21:
            landing = cmd

    landing_location = LocationGlobal(landing.x, landing.y, landing.z) 

    print(landing_location)

    while (True):
        
        if (len(list(cmds)) == cmds.next and getDistance(copter.vehicle.location.global_frame,landing_location) <= 5):
            print "@landing Location"
            break
        time.sleep(1)
    
    while (copter.getMode() != "GUIDED"):
        copter.setMode("GUIDED")

    
    prev_height = 50
    while (copter.pos_alt_rel > 8.0):
        height = copter.pos_alt_rel
        vz = kpz*10*(height - 7.5) + kdz*10*(height - prev_height)
        copter.setOffsetVelocity(0, 0, vz)
        prev_height = height
        time.sleep(0.1)

    prev_data = [0, 0, 0]

    print("\n\n*******waiting for vision system********\n\n")

    z = 10
    while (z > 0.2):

        raw_data = getTagInfo(tagID)
        timeoutCount = 0

        
        if (((raw_data == "TIMEOUT") and (~foundTag) and (seeked)) or timeoutCount == 5):
            copter.setMode("RTL")
            break

        elif (raw_data == "TIMEOUT" and (~foundTag)):
            #seek()
            seeked = True
            continue
 
        elif (raw_data == "TIMEOUT"):
            data = predictionOf(prev_data, velocities)
            velocities = calcPID(data, prev_data, 0.1)
            velocities[2] = 0
            timeoutCount += 1

        
        elif (len(raw_data) > 0):
            timeoutCount = 0
            foundTag = True

            data = [float(i.strip().replace("\x00","")) for i in raw_data.split(",")]
            raw_data = None
            print data
            
            velocities = calcPID(data, prev_data, 0.1)


        prev_data = data
        z = data[2]
        copter.setOffsetVelocity(velocities[0], velocities[1], velocities[2])

    
    copter.setMode("LAND")



def getTagInfo(tagID):
    readFifo = open(FIFO_R, 'r')
    raw_data = readFifo.read().strip().split(",")

    return ",".join(raw_data[2:5])


def calcPID(curr_data, prev_data, max_err):
    x_dot = curr_data[0] - prev_data[0]
    y_dot = curr_data[1] - prev_data[1]
    z_dot = curr_data[2] - prev_data[2]

    vx = kp*curr_data[0]+kd*x_dot
    vy = kp*curr_data[1]+kd*y_dot
    vz = kp*curr_data[2]+kd*z_dot

    if ((curr_data[0]**2 + curr_data[1]**2) > max_err**2):
        vz = 0 

    return [vx, vy, vz] 


if __name__=='__main__':
    main()
