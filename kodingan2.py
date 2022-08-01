from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600)

oncePrinted = False

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

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
    print("x distance to next waypoint : %s" %s(lat))
    print("y distance to next waypoint : %s " %s(lon))
    print("z distance to next waypoint : %s" %s(alt))
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

while True:
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot is controlling")
            oncePrinted = True
    elif (vehicle.mode == VehicleMode('AUTO')):
        if(oncePrinted == False):
            print("Next waypoint : %s" %(nextwaypoint))
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
            oncePrinted = True
            if (distance_to_current_waypoint == 0):
                Vehicle.mode = VehicleMode("LOITER")
                print("Loiter on")
    time.sleep(1)
