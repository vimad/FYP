#! /usr/bin/python

from communication.quadcopter import Quadcopter
import time

connection_string = 'udp:127.0.0.1:14551'
baud = 115200


print('Connecting to the vehicle ...')

copter = Quadcopter(connection_string = connection_string, baud = baud)

copter.clearMission()

copter.arm()
time.sleep(1)
copter.takeoff()

copter.setMode("LOITER")
time.sleep(5)
copter.setMode("GUIDED")
print(copter.flight_mode)

copter.setOffsetVelocity(1,2,5)
copter.gotoLocation(50,20)
copter.gotoLocation(10,100)
copter.gotoLocation(30,20)

#copter.gotoLocation(4,0)
#copter.gotoLocation(-2,2*math.sqrt(3.0))
#copter.gotoLocation(2,2*math.sqrt(3.0))
#copter.gotoLocation(2,2*math.sqrt(3.0))
#copter.gotoLocation(2,2*math.sqrt(3.0))
#copter.gotoLocation(2,2*math.sqrt(3.0))

copter.setMode('RTL')
