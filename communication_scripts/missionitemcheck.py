#! /usr/bin/python

from dronekit import LocationGlobal

from util.util import rotmat,rotmat_inverse,mul_mat,getCordinatesInEarthFrame

from communiction.com_protocol import getLocation_meters,getDistance,getBearing,Quadcopter

import os,time,math,sys,errno

connectString = 'udp:127.0.0.1:14551'
baud = 115200

copter = Quadcopter(connection_string = connectString, baud = baud)

cmds = copter.download_mission()

lx = cmds[-1].x
ly = cmds[-1].y
lz = cmds[-1].z

landing_location = LocationGlobal(lx, ly, lz)

while (copter.is_armed()):
    if (len(list(cmds)) == cmds.next and getDistance(copter.vehicle.location.global_frame,landing_location) <= 2.5):
        print "Shit got real"
        break
    time.sleep(1)

