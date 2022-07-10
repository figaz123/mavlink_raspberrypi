from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import numpy as np
from datetime import datetime
from time import sleep


# Connect to the Vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
# vehicle = connect('127.0.0.1:14550', wait_ready=True)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

#def servo(channel, sv):
    # input the message
#    msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
#                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
#                                                      0,  # konfirmasi
#                                                      channel,  # pin relay pada AUX OUT 3
#                                                      sv,  # pwm value
#                                                      0, 0, 0, 0, 0)  # param 1 ~ 5 ga dipake
    # send command to vehicle
#    vehicle.send_mavlink(msg)
#    vehicle.flush()

while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot it controlling")
            oncePrinted = True
    else:
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
            