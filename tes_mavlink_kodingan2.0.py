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


def send_nav_velocity(velocity_x, velocity_y, velocity_z): #jadi ini itu fungsi buat ngatur kecepatan 
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot it controlling")
            oncePrinted = True
    elif(vehicle.mode == VehicleMode("GUIDED")):
        oncePrinted = False
        nextwaypoint = vehicle.commands.next
        if(nextwaypoint == 1):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 1')
        elif (nextwaypoint == 2):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 2')
        elif(nextwaypoint == 3):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            vehicle.mode = VehicleMode('GUIDED')
            print('waypoint 3')
        elif(nextwaypoint == 4):
            vehicle.mode = VehicleMode('LOITER')
            time.sleep(10)
            print('go to waypoint 4')
            vehicle.mode = VehicleMode('GUIDED')
            
            
