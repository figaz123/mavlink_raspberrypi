#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative



import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None



if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()



print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  

   
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


#def nav_loiter_waypoint():


arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 20 seconds ...")
point1 = LocationGlobalRelative(-7.1430311, 107.508456, 2)
vehicle.simple_goto(point1, groundspeed=0.6)


time.sleep(20)

print("Going towards second point for 20 seconds (groundspeed set to 0.6 m/s) ...")
point2 = LocationGlobalRelative(-7.1431615, 107.507945, 2)
vehicle.simple_goto(point2, groundspeed=0.6)

print("Going towards third waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point3 = LocationGlobalRelative(-7.1431109, 107.5079463, 2)
vehicle.simple_goto(point3, groundspeed=0.6)
time.sleep(20)

print("Going towards forth waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point4 = LocationGlobalRelative(-7.1430763, 107.5084586, 2)
vehicle.simple_goto(point4, groundspeed=0.6)
time.sleep(20)

print("Going towards fifth waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point5 = LocationGlobalRelative(-7.1431615, 107.507945, 2)
vehicle.simple_goto(point5, groundspeed=0.6)
time.sleep(20)

#print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")


print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()
