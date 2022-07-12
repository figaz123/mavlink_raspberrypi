from __future__ import print_function
import time
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import numpy as np
from datetime import datetime
from time import sleep
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

oncePrinted = False

while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot it controlling")
            oncePrinted = True
    elif(vehicle.mode == VehicleMode("GUIDED")):
        oncePrinted = False
        nextwaypoint = vehicle.commands.next
        if(nextwaypoint == 1):
            print('waypoint 1')
        elif (nextwaypoint == 2):
            print('waypoint 2')
        elif(nextwaypoint == 3):
            print('waypoint 3')
        elif(nextwaypoint == 4):
            print('waypoint 4')
            
if sitl:
    sitl.stop()
