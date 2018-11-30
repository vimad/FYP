#! /usr/bin/python


#############################################################################################################
#############################################################################################################
#**************************                                                                                 #
#**************************                                                                                 #
#**************************                                                                                 #
#->This module will provide an object for modelling a quadcopter and its functionalities                    #
#->some essential methods used for them are implemented in ./util.py module                                 #
#                                                                                                           #
#                                                                                                           #
#############################################################################################################

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil
from util import *

import time, math
import os


######################################################################################################################
#######################---------Quadcopter model with basic functionalities-------------------########################
class Quadcopter:

    def __init__(self, connection_string = None, baud = 57600):
        if not connection_string is None:
            self.connect(connection_string = connection_string, baud = baud)    #- Using the provided connection string and baud rate

        else:
            raise("Error: A valid connection string is not provided")
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

        self.flight_mode        = ''        #- []       Autopilot flight mode

        self.mission            = self.vehicle.commands #-- mission items

        self.location_home      = LocationGlobalRelative(0, 0, 0) #- LocationRelative type home
        self.location_current   = LocationGlobalRelative(0, 0, 0) #- LocationRelative type current position

        self.setupListeners()


    def connect(self, connection_string, baud):

        '''This will return a vehicle object connected to the vehicle with given connection string'''

        self.vehicle = connect(connection_string, wait_ready = True, heartbeat_timeout = 60, baud = baud)
        self.setupListeners()

        print('connection completed..!')


    def setupListeners(self):

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


    def isArmed(self):
        '''Returns true if vehicle if armed false otherwise'''
        return self.vehicle.armed



    def arm(self):
        '''Arm the quadcopter'''
        self.vehicle.armed = True



    def disarm(self):
        '''disarm the quadcopter'''
        self.vehicle.armed = False



    def setMode(self, mode):
        '''
           Change the autopilot mode to "mode"
           Return true if successfully changed false otherwise
        '''
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

        ''' returns the current mode of quadcopter as a block capital string'''

        self.flight_mode = self.vehicle.mode
        
        return self.vehicle.mode






    def takeoff(self, altitude = 20, pitch = 0):
        '''
            Autonomous takeoff with a given altititude and a pitch
            return true if done without any error, false otherwise
        '''

        #---------------add takeoff point and altitude if not set already--------------------#
        self.missionAddTakeoff(altitude=1.5*altitude, pitch=pitch)
        #------------------------------------------------------------------------------------#

        
        #-------wait until armable-----------------------------------------------------------#
        while (not self.vehicle.is_armable):
            print ('Waiting until vehicle is armable...')
            time.sleep(1.0)
        #------------------------------------------------------------------------------------#


        #--------------------------wait until GPS Lock---------------------------------------#
        while (self.pos_lat == 0.0):
            time.sleep(0.5)
            print ('waiting for GPS lock...')
        #------------------------------------------------------------------------------------#

        
        #--------------------------set current location as home location---------------------#
        self.location_home = LocationGlobalRelative(self.pos_lat, self.pos_lon, altitude)
        #------------------------------------------------------------------------------------#


        #--------------------------Arm the quadcopter----------------------------------------#
        noOfTries = 0

        while (not self.vehicle.armed):
            self.arm()
            noOfTries += 1
            time.sleep(2.0)

            if (noOfTries > 5):
                print ('Cannot arm')
                break
        #-------------------------------------------------------------------------------------#
        

        

        #--------------------------------Change mode to Auto----------------------------------#
        if (self.vehicle.armed):
            print ('Armed!!')
            while (not (self.getMode() == "GUIDED")):
                self.setMode("GUIDED")
                time.sleep(0.1)
            
            #---------------------------Takeoff-----------------------------------------------#
            self.vehicle.simple_takeoff(altitude)
            #---------------------------------------------------------------------------------#



            #------------------Check until altitude reached-----------------------------------#
            while (self.pos_alt_rel <= altitude - 1.0):
                print ("Altitude = %0.0f", self.pos_alt_rel)
                time.sleep(0.5)
            #---------------------------------------------------------------------------------#


            #--------------------Change mode to Guided-----------------------------------------#
            print('Change mode to GUIDED')
            while (not (self.getMode() == "GUIDED")):
                self.setMode("GUIDED")
                time.sleep(0.1)
            #----------------------------------------------------------------------------------#

            time.sleep(1.0)

            #---------------------sending to home location-------------------------------------#
            print('sending to home')
            self.vehicle.simple_goto(self.location_home)
            #----------------------------------------------------------------------------------#

            return True

        return False









    def missionAddTakeoff(self, altitude, pitch = 0, heading = None):
        '''
            Add a takeoff item into the mission if not set already
        '''
        
        #-------------------------Set default heading-------------------------------------#
        if heading is None: heading = self.att_heading_deg
        #---------------------------------------------------------------------------------#
       
        #-------------------------Download the current mission-----------------------------#
        self.downloadMission()
        #-- save the mission: copy in the memory
        tmp_mission = list(self.mission)
        #-----------------------------------------------------------------------------------#
        
        print tmp_mission.count


        #-------------------------Check availability of a mission---------------------------#
        is_mission  = False

        if len(tmp_mission) >= 1:
            is_mission = True

            #------------------------Display mission items----------------------------------#
            print("Current mission:")
            for item in tmp_mission:
                print item
            #-------------------------------------------------------------------------------#


        #---------------------------Adding takeoff if takeoff not in mission----------------#
        if ((is_mission) and (tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)):
            print ("Takeoff already in the mission")
        else:
            print("Takeoff not in the mission: adding")
            self.clearMission()
            takeoff_item = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, pitch,  0, 0, heading, 0,  0, altitude)
            self.mission.add(takeoff_item)
            for item in tmp_mission:
                self.mission.add(item)
            self.vehicle.flush()
            print("#-------------Takeoff Added------------------#")
        #-----------------------------------------------------------------------------------#

        #--------------------------Display mission items------------------------------------#
        self.downloadMission()
        for i in self.mission:
            print i
        #-----------------------------------------------------------------------------------#

        print('\n\n\n#---------------------------Takeoff Added---------------------------#\n\n\n')








    def downloadMission(self):
        '''returns the full mission as a object'''
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready() # wait until download is complete.
        self.mission = self.vehicle.commands
	return self.vehicle.commands



    def clearMission(self):
        '''removes the current mission from quadcopter'''
        #----------------remove items from pixhawk----------------------------------------#
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()

        #-----------------update self.mission with current mission------------------------#
        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()



    def gotoLocation(self, northDistance, eastDistance, waypointRadius = 2):
        '''navigates quadcopter to a given distance in north and east axes'''
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = getLocation_meters(currentLocation, northDistance, eastDistance)
        targetDistance = getDistance(currentLocation, targetLocation)
        self.vehicle.simple_goto(targetLocation)

        while (getDistance(self.vehicle.location.global_relative_frame, targetLocation) > waypointRadius):
            print("goto distance", self.airspeed, self.groundspeed, getDistance(self.vehicle.location.global_relative_frame, targetLocation))
            time.sleep(0.5)


       


    def setOffsetVelocity(self, vx, vy, vz):
        '''send x,y,z velocities to the quadcopter'''
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
###############################################################################################################################################
###############################################################################################################################################

