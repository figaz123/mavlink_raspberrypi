#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import numpy as np
from datetime import datetime


#import argparse
#parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
#parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
#args = parser.parse_args()
#connection_string = args.connect
#sitl = None

#if not connection_string:
#    import dronekit_sitl
#    sitl = dronekit_sitl.start_default()
#    connection_string = sitl.connection_string()

#print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
vel_x = 0
vel_y = 0
vel_z = 0

pointauth = 2
settime = 0
mode_changed = 0
count = 0
once = False
settime = 0
oncePrinted = False
dropnextwaypoint = 0 

#connection_string = args.connect


#def send_nav_velocity(velocity_x, velocity_y, velocity_z): #jadi ini itu fungsi buat ngatur kecepatan 
    # create the SET_POSITION_TARGET_LOCAL_NED command
#    msg = vehicle.message_factory.set_position_target_local_ned_encode(
#        0,  # time_boot_ms (not used)
#        0, 0,  # target system, target component
#        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
#        0b0000111111000111,  # type_mask (only speeds enabled)
#        0, 0, 0,  # x, y, z positions (not used)
#        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
#        0, 0, 0,  # x, y, z acceleration (not used)
#        0, 0)  # yaw, yaw_rate (not used)
    # send command to vehicle
#    vehicle.send_mavlink(msg)
#    vehicle.flush()
    
while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        #print('pilot controlling')
        if(oncePrinted == False):
            print("pilot it controlling")
            oncePrinted = True
    else:
        print('start')
        oncePrinted = False
        nextwaypoint = vehicle.commands.next
        if(nextwaypoint == 2):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 2')
        elif (nextwaypoint == 3):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 3')
        elif(nextwaypoint == 4):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 4')
        elif(nextwaypoint == 5):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            print('waypoint 5')
            vehicle.mode = VehicleMode('GUIDED')
        elif(nextwaypoint == 6):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            print('waypoint 6')
            print('finish')


#print("Close vehicle object")
#vehicle.close()
    
#if sitl:
   # sitl.stop()




#def nav_loiter_waypoint():
#print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")





