#! /usr/bin/python

from dronekit import LocationGlobal

from communication.quadcopter import Quadcopter
from communication.util import getDistance
from util.util import *
import numpy as np

import os, sys, time, math, errno


tagID = 13

kp = 0.1
kd = 0.05
ki = 0

kpz = 0.05
kdz = 0.025

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
    copter.downloadMission()
    
    foundTag = False
    seeked = False

    #while ((not copter.isArmed()) or (copter.getMode() != "AUTO")):
    #    pass
   
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

    
    prev_height = 25
    copter.setOffsetVelocity(0,0,0)
    time.sleep(4)

    while (copter.pos_alt_rel > 8.0):
        height = copter.pos_alt_rel
        vz = kpz*10*(height - 7.5) + kdz*10*(prev_height - height)
        copter.setOffsetVelocity(0, 0, vz)
        prev_height = height
        time.sleep(0.1)

    prev_data = [0, 0, 0]
    
    sendVisionRequest("start")
    print("\n\n*******waiting for vision system********\n\n")
    
    t1 = time.time()
    
    filename = "datalog_"+time.strftime("%Y-%m-%d %H:%M:%S")
    z = 10
    while (z > 1.0):

        raw_data = getTagInfo(tagID)
        timeoutCount = 0
        
        roll=copter.att_roll_deg*np.pi/180
        pitch=copter.att_pitch_deg*np.pi/180
        yaw=0

        
        if (((raw_data == "TIMEOUT") and (not foundTag) and (seeked)) or timeoutCount == 5):
            copter.setMode("RTL")
            break

        elif (raw_data == "TIMEOUT" and (not foundTag)):
            prev_data = [0,0,0]
            seeked = True
            continue
 
        elif (raw_data == "TIMEOUT"):
            print("In prediction step")
            corrected_cords = predictionOf(prev_cords, corrected_velocities, time.time()-t1)
            corrected_velocities = getVectorInDroneFrame(roll, pitch, velocities)
            copter.setOffsetVelocity(corrected_velocities[0], corrected_velocities[1], corrected_velocities[2])
            print("predicted",velocities)
            prev_cords = corrected_cords
            
            timeoutCount += 1

        
        elif (len(raw_data) > 0):
            print("Normal state")
            try:
                data = [float(i.strip().replace("\x00","")) for i in raw_data.split(",")]
                corrected_cords = getVectorInEarthFrame(roll,pitch,data)
                timeoutCount = 0
                foundTag = True
                seeked = False
            except ValueError as ve:
                continue

            raw_data = None
            print data
            
            if (z > 1.6):
                velocities = calcPID(corrected_cords, prev_cords, (z-1)/3.0)
            else:
                velocities = calcPID(corrected_cords, prev_cords, 0.2)


        prev_cords = corrected_cords
        z = corrected_cords[2]
        corrected_velocities = getVectorInDroneFrame(roll, pitch, velocities)
        copter.setOffsetVelocity(corrected_velocities[0], corrected_velocities[1], corrected_velocities[2])
        t1 = time.time()
        print("normal",corrected_velocities)
        #logFile = open(filename,"a+")
        #logFile.write(raw_data)
        #logfile.close()

    
    copter.setMode("LAND")



def getTagInfo(tagID):
    t1 = time.time()
    readFifo = open(FIFO_R, 'r')
    while (True):
        raw_data = readFifo.read().strip().split(",")
        if (raw_data[0] == '2'):
            break
    else:
        readFifo.close()
        return "TIMEOUT"

    readFifo.close()

    return ",".join(raw_data[2:5])

def sendVisionRequest(message):
    writeFifo = open(FIFO_W, "w")
    writeFifo.write(message)
    writeFifo.close()


def calcPID(curr_data, prev_data, max_err):
    x_dot = curr_data[0] - prev_data[0]
    y_dot = curr_data[1] - prev_data[1]
    z_dot = curr_data[2] - prev_data[2]

    vx = kp*curr_data[0]+kd*x_dot
    vy = kp*curr_data[1]+kd*y_dot
    vz = kpz*curr_data[2]+kdz*z_dot

    if ((curr_data[0]**2 + curr_data[1]**2) > max_err**2):
        vz = 0 

    return [vx, vy, vz] 

def predictionOf(prev_data, velocities, time_diff):
    data = [prev_data[i]-velocities[i]*time_diff for i in range(3)]
    return data
    


if __name__=='__main__':
    main()
