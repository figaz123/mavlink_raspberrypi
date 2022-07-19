#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative



import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
#parser.add_argument('--connect',
#                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
#args = parser.parse_args()

#connection_string = args.connect
#sitl = None

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600)

vehicle.mode = VehicleMode("GUIDED")
    
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

#def nav_loiter_waypoint():

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 20 seconds ...")
point1 = LocationGlobalRelative(-6,9701293, 107,6290575, 2)
vehicle.simple_goto(point1, groundspeed=0.6)
vehicle.mode = VehicleMode("LOITER")
time.sleep(20)
vehicle.mode=VehicleMode("GUIDED")

print("Going towards second point for 20 seconds (groundspeed set to 0.6 m/s) ...")
point2 = LocationGlobalRelative(-6.9702824, 107.6290642, 2)
vehicle.simple_goto(point2, groundspeed=0.6)
vehicle.mode = VehicleMode("LOITER")
time.sleep(20)
vehicle.mode=VehicleMode("GUIDED")

print("Going towards third waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point3 = LocationGlobalRelative(-6.9702824, 107.6291835, 2)
vehicle.simple_goto(point3, groundspeed=0.6)
vehicle.armed = VehicleMode("LOITER")
time.sleep(20)
vehicle.mode=VehicleMode("GUIDED")

print("Going towards forth waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point4 = LocationGlobalRelative(-6.9701093, 107.6291741, 2)
vehicle.simple_goto(point4, groundspeed=0.6)
vehicle.armed = VehicleMode("LOITER")
time.sleep(20)
vehicle.mode=VehicleMode("GUIDED")

print("Going towards fifth waypoint for 20 seconds (groundspeed set to 0.6 m/s) ...")
point5 = LocationGlobalRelative(-6.9700587, 107.6290615, 2)
vehicle.simple_goto(point5, groundspeed=0.6)
vehicle.armed = VehicleMode("LOITER")
time.sleep(20)
vehicle.mode=VehicleMode("GUIDED")

#print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")


print("Close vehicle object")
vehicle.close()