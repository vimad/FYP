#! /usr/bin/python

from communication.quadcopter import Quadcopter

connection_string = '/dev/ttyS0'
baud = 57600

print('Connecting to the vehicle ...')
copter = Quadcopter(connection_string = connection_string, baud = baud)

#copter.mission_add_takeoff(2,0,0)
copter.takeoff(2,0)

copter.setMode("GUIDED")
copter.setMode("LAND")

while(copter.pos_alt_rel >= 0.2):
    pass

copter.disarm()

print(copter.flight_mode)

copter.setMode('RTL')

