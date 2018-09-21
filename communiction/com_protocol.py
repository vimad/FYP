#!/usr/bin/python

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil
import dronekit_sitl

import time, math
import numpy as np

import os
import errno

import argparse

def getLocation_meters(originalLocation, dNorth, dEast, isGlobal = False ):
    
    earth_radius = 6378137.0    #- radius of earth (spherical model)
    
    dLat = dNorth/earth_radius  #- difference in lattitude axis
    dLon = dEast/(earth_radius*math.cos(math.pi*originalLocation.lat/180)) #-difference in longitudinal axis
   
    newLat = originalLocation.lat + dLat*180.0/math.pi
    newLon = originalLocation.lon + dLon*180.0/math.pi

    if isGlobal:
        return LocationGlobal(newLat, newLon,originalLocation.alt)
    else:
        return LocationGlobalRelative(newLat, newLon,originalLocation.alt)


def getDistance(location1, location2):
     y_diff = location2.lat - location1.lat
     x_diff = location2.lon - location1.lon
     
     return math.sqrt(x_diff**2 + y_diff**2) * 1.113195e5
 

def getBearing(location1, location2):
     
     x_diff = location2.lon - location1.lon
     y_diff = location2.lat - location1.lat
     
     bearing = 90.0 + math.atan2(-y_diff, x_diff)*57.2957795
     
     return bearing if (bearing >= 0) else bearing + 360.0



class Quadcopter:

    def __init__(self, connection_string = None, vehicle = None):
        if not vehicle is None:
            self.vehicle = vehicle          #- Using the provided vehicle

        elif not connection_string is None:
            self._connect(connection_string)    #- Using the provided Connection string

        else:
            raise("Error: A valid vehicle or a connection string is not provided")
            return


        self.airspeed           = 0.0       #- [m/s]    airspeed
        self.groundspeed        = 0.0       #- [m/s]    ground speed

        self.pos_lat            = 0.0       #- [deg]    latitude
        self.pos_lon            = 0.0       #- [deg]    longitude
        self.pos_alt_rel        = 0.0       #- [m]      altitude relative to takeoff
        self.pos_alt_abs        = 0.0       #- [m]      above mean sea level

        self.att_roll_deg       = 0.0       #- [deg]    roll
        self.att_pitch_deg      = 0.0       #- [deg]    pitch
        self.att_heading_deg    = 0.0       #- [deg]    magnetic heading

        self.wind_dir_to_deg    = 0.0       #- [deg]    wind direction (where it is going)
        self.wind_dir_from_deg  = 0.0       #- [deg]    wind coming from direction
        self.wind_speed         = 0.0       #- [m/s]    wind speed

        self.climb_rate         = 0.0       #- [m/s]    climb rate
        self.throttle           = 0.0       #- [ ]      throttle (0-100)

        self.flight_mode            = ''        #- []       Autopilot flight mode

        self.mission            = self.vehicle.commands #-- mission items

        self.location_home      = LocationGlobalRelative(0,0,0) #- LocationRelative type home
        self.location_current = LocationGlobalRelative(0,0,0) #- LocationRelative type current position

        self._setup_listeners()


    def _connect(self, connection_string):

        self.vehicle = connect(connection_string, wait_ready = True, heartbeat_timeout = 60)
        self._setup_listeners()

        print('connection completed..!')


    def _setup_listeners(self):

        print('Setting up listeners...')

        @self.vehicle.on_message('ATTITUDE')   
        def listener(vehicle, name, message):          #--- Attitude
            self.att_roll_deg   = math.degrees(message.roll)
            self.att_pitch_deg  = math.degrees(message.pitch)
            self.att_heading_deg = math.degrees(message.yaw)%360
            missionitem=self.vehicle.commands[self.vehicle.commands.next-1] #commands are zero indexed


        @self.vehicle.on_message('GLOBAL_POSITION_INT')       
        def listener(vehicle, name, message):          #--- Position / Velocity                                                                                                             
            self.pos_lat        = message.lat*1e-7
            self.pos_lon        = message.lon*1e-7
            self.pos_alt_rel    = message.relative_alt*1e-3
            self.pos_alt_abs    = message.alt*1e-3
            self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel)
               
                
        @self.vehicle.on_message('VFR_HUD')
        def listener(vehicle, name, message):          #--- HUD
            self.airspeed       = message.airspeed
            self.groundspeed    = message.groundspeed
            self.throttle       = message.throttle
            self.climb_rate     = message.climb 
                
        @self.vehicle.on_message('WIND')
        def listener(vehicle, name, message):          #--- WIND
            self.wind_speed         = message.speed
            self.wind_dir_from_deg  = message.direction % 360
            self.wind_dir_to_deg = (self.wind_dir_from_deg + 180) % 360

        print('Setting up listners completed..!')
        return



    def arm(self):
        self.vehicle.armed = True



    def disarm(self):
        self.vehicle.armed = False



    def setMode(self, mode):
        t_0 = time.time()

        try:
            vMode = VehicleMode(mode)
        except:
            return False

        while (self.getMode() != vMode):
            self.vehicle.mode = vMode
            time.sleep(0.2)

            if (time.time() > t_0 + 5):
                return False

        return True




    def getMode(self):

        self.flight_mode = self.vehicle.mode
        
        return self.vehicle.mode

    

    def takeoff(self, altitude = 20, pitch = 0):
        self.mission_add_takeoff(altitude=1.5*altitude, pitch=pitch)

        while (not self.vehicle.is_armable):
            print ('Waiting until vehicle is armable...')
            time.sleep(1.0)

        while (self.pos_lat == 0.0):
            time.sleep(0.5)
            print ('waiting for GPS lock...')

        self.location_home = LocationGlobalRelative(self.pos_lat, self.pos_lon, altitude)

        noOfTries = 0

        while (not self.vehicle.armed):
            self.arm()
            noOfTries += 1
            time.sleep(2.0)

            if (noOfTries > 5):
                print ('Cannot arm')
                break


        if (self.vehicle.armed):
            print ('Armed!!')
            self.setMode("GUIDED")
            self.vehicle.simple_takeoff(altitude)



            while (self.pos_alt_rel <= altitude - 1.0):
                print ("Altitude = %0.0f", self.pos_alt_rel)
                time.sleep(2.0)
                
            print('Change mode to GUIDED')
            self.setMode("GUIDED")

            time.sleep(1.0)

            print('sending to home')
            self.vehicle.simple_goto(self.location_home)

            return True

        return False


    def mission_add_takeoff(self, altitude, pitch, heading = None):
        if heading is None: heading = self.att_heading_deg
        
        self.download_mission()
        #-- save the mission: copy in the memory
        tmp_mission = list(self.mission)
        
        print tmp_mission.count
        is_mission  = False
        if len(tmp_mission) >= 1:
            is_mission = True
            print("Current mission:")
            for item in tmp_mission:
                print item
            #-- If takeoff already in the mission, do not do anything
            
        if is_mission and tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print ("Takeoff already in the mission")
        else:
            print("Takeoff not in the mission: adding")
            self.clear_mission()
            takeoff_item = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, pitch,  0, 0, heading, 0,  0, altitude)
            self.mission.add(takeoff_item)
            for item in tmp_mission:
                self.mission.add(item)
            self.vehicle.flush()
            print(">>>>>Done")

        self.download_mission()
        for i in self.mission:
            print i

        print('\n\n\mtakeoff complete...\n\n\n')

    def download_mission(self):
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready() # wait until download is complete.
        self.mission = self.vehicle.commands



    def clear_mission(self):
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()


        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()



    def gotoLocation(self, northDistance, eastDistance):
        
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = getLocation_meters(currentLocation, northDistance, eastDistance)
        targetDistance = getDistance(currentLocation, targetLocation)
        self.vehicle.simple_goto(targetLocation)

        while (getDistance(self.vehicle.location.global_relative_frame, targetLocation) >= 1.0):
            print("goto distance", self.airspeed, self.groundspeed, getDistance(self.vehicle.location.global_relative_frame, targetLocation))
            time.sleep(0.5)


       


    def setOffsetVelocity(self, vx, vy, vz, t):
        print('\n\n\nVelocity...\n')
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0)

        self.vehicle.send_mavlink(msg)

        t_0 = time.time()

        while (time.time() - t_0 <= t):
            print("groundspeed: %f", self.groundspeed)
            print("Airspeed: %f", self.airspeed)
            time.sleep(0.5)

        self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0)

        self.vehicle.send_mavlink(msg)

        print('\n\n\nVelocity mode over...\n')





def test():

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    #sitl = dronekit_sitl.start_default()
    #connection_string = sitl.connection_string()
    #connection_string = 'udp:127.0.0.1:14550'
    '''
    #connection_string = '/dev/ttyAMA0'
    fifo1 = '/tmp/server_to_client_fifo'
    fifo2 = '/tmp/client_to_server_fifo'


    try:
        os.mkfifo(fifo1)
        os.mkfifo(fifo2)

    except OSError as oe:
        if oe.errno != errno.EEXIST:
            raise
    '''


    print('Connecting to the vehicle ...')
    copter = Quadcopter(connection_string = connection_string)
    
    copter.arm()
    copter.takeoff()

    copter.setMode("GUIDED")
    print(copter.flight_mode)

    copter.setOffsetVelocity(1,2,5,10)
    copter.gotoLocation(50,20)
    copter.gotoLocation(10,100)
    copter.gotoLocation(30,20)

    copter.gotoLocation(4,0)
    copter.gotoLocation(-2,2*math.sqrt(3.0))
    copter.gotoLocation(2,2*math.sqrt(3.0))
    copter.gotoLocation(2,2*math.sqrt(3.0))
    copter.gotoLocation(2,2*math.sqrt(3.0))
    copter.gotoLocation(2,2*math.sqrt(3.0))


    copter.setMode('RTL')


if __name__ == '__main__':
    test()
