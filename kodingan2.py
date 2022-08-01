from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600)

oncePrinted = False
nextwaypoint = 0
a = float(0)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    print("distance on")
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    print("x distance to next waypoint : %s" %(lat))
    print("y distance to next waypoint : %s " %(lon))
    print("z distance to next waypoint : %s" %(alt))
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

while True:
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot is controlling")
            oncePrinted = True
    else:
        if(oncePrinted == False):
            print("Mission start")
            print("Next waypoint : %s" %(nextwaypoint))
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
            #oncePrinted = True
            #print(type(distance_to_current_waypoint))
            a = float(distance_to_current_waypoint())
            print(type(a))
            if (a < 2):
               Vehicle.mode = VehicleMode("LOITER")
               print("Loiter on")
               time.sleep(5)
               Vehicle.mode = VehicleMode("AUTO")
    time.sleep(1)
