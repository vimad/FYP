#! /usr/bin/python

import math
from dronekit import LocationGlobal, LocationGlobalRelative

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


