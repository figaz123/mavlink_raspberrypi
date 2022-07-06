from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command 
from pymavlink import mavutil 
import cv2 
import numpy as np 
import math 
from datetime import datetime

cap = cv2.VideoCapture(0)
#cap.set(CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')
cap.set(3, 640)# width resolution
cap.set(4, 360)# heigh resolution

vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640,360))

# Connect to the Vehicle
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)

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


def servo(channel, sv):
    # input the message
    msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                      0,  # konfirmasi
                                                      channel,  # pin relay pada AUX OUT 3
                                                      sv,  # pwm value
                                                      0, 0, 0, 0, 0)  # param 1 ~ 5 ga dipake
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def move_target_drop(channel, pwm, waypoint):
    global dropnextwaypoint
    if (dropnextwaypoint == waypoint):
        servo(channel,pwm)
while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("pilot it controlling")
            oncePrinted = True
    else:
        oncePrinted = False
        nextwaypoint = vehicle.commands.next
        frame, img = cap.read()


       # cv2.rectangle(img, (230, 100), (430, 280), (0, 255, 0), 5)
	pts = np.array([[225, 220], [225, 295],
                [275, 340], [350, 340],
                [400, 295], [400, 220],
                [350, 175], [275, 175],[225, 220]],
               np.int32)

 
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, (255, 0, 0), 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        orange_lower = np.array([101,50,38], np.uint8)
        orange_upper = np.array([110,255,255], np.uint8)

        # bluring image
        blur = cv2.GaussianBlur(hsv, (15, 15), cv2.BORDER_DEFAULT)


        orange = cv2.inRange(blur, orange_lower, orange_upper)
        kernal = np.ones((5, 5), "uint8")

        res = cv2.bitwise_and(img, img, mask=orange)

        # Tracking Colour (orange)
        _, contours, _= cv2.findContours(orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # contours_roi,hierarchy_roi=cv2.findContours(orange_roi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 2000):
                c = max(contours, key=cv2.contourArea)
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print (cX, cY)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
		servo(10,1100)
                #if (nextwaypoint == 3):
                #move_target_drop(10, 1100, nextwaypoint)
                # if (nextwaypoint == 4):
                #     move_target_drop(cX, cY, 5, 2400, nextwaypoint)
                # if (nextwaypoint == 5):
                #     move_target_drop(cX, cY, 6, 1700, nextwaypoint)
                # if (nextwaypoint == 6):
                #     move_target_drop(cX, cY, 6, 2400, nextwaypoint)
                # if (nextwaypoint == 7):
                #     move_target_drop(cX, cY, 7, 1700, nextwaypoint)
                # if (nextwaypoint == 8):
                #     move_target_drop(cX, cY, 7, 2400, nextwaypoint)

        #cv2.imshow("Orange",res)
        #output.write(frame)

    if cv2.waitKey(10) & 0xFF == 27:
        break
cv2.destroyAllWindows()
