#! /usr/bin/python

from util.util import rotmat,rotmat_inverse,mul_mat,getCordinatesInEarthFrame

from communiction.com_protocol import getLocation_meters,getDistance,getBearing,Quadcopter

import os,time,math,sys,errno


TARGET_ID = 13

mode = "SIMULATION"

kp = 1
kd = 0.5
ki = 0


FIFO_1 = '/tmp/server_to_client_fifo'
FIFO_2 = '/tmp/client_to_server_fifo'

if (mode != "SERIAL"):
    connectString = '/dev/ttyS0'
    baud = 57600
else:
    connectString = 'udp:127.0.0.1:14551'
    baud = 115200


try:
    os.mkfifo(FIFO_1)
    os.mkfifo(FIFO_2)

except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise


def main():


    copter = Quadcopter(connection_string = connectString, baud = baud)

    foundTag = False
    seeked = False

    while (copter.getMode() != "LAND"):
        pass
    else:
        while(copter.getMode() != "GUIDED"):
            copter.setMode("GUIDED")
            time.sleep(0.1)

    prev_height = 50
    while (copter.pos_alt_rel > 8.0):
        height = copter.pos_alt_rel
        vz = kpz*(height - 8) + kdz*(height - prev_height)
        time.sleep(0.1)

    prev_location = [0, 0, 0]

    while (copter.pos_alt_rel >= 0.2):

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
            #getPrediction()
            #copter.goto_location()
            timeoutCount += 1

        elif (len(raw_data) > 0):
            timeoutCount = 0
            foundTag = True

            data = map(float, raw_data.split(','))
            raw_data = None
            
            velocities = calcPID(data, prev_data, 0.1)

            copter.setOffsetVelocity(velocities[0], velocities[1], velocities[2])

    if (copter.pos_alt_rel <= 0.2):
        setMode("LAND")


def calcPID(curr_data, prev_data, max_err):
    x_dot = curr_data[0] - prev_data[0]
    y_dot = curr_data[1] - prev_data[1]
    x_dot = curr_data[2] - prev_data[2]
   
    vx = kp*curr_data[0]+kd*x_dot
    vy = kp*curr_data[1]+kd*y_dot
    vz = kp*curr_data[2]+kd*z_dot

    if ((curr_data[0]**2 + curr_data[1]**2) > max_err):
        vz = 0

    return [vx, vy, vz]

if __name__=='__main__':
    main()
