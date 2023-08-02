from threading import Thread, Event
from time import sleep
from dt_apriltags import Detector
import cv2
import matplotlib.pyplot as plt
import numpy as np
from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from processing import *
from lane_processing import *

predator = True

# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
pid_horizontal = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

vertical_power = 0
lateral_power = 0

heading_power = 0
lane_lateral_power = 0

at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

def _get_frame():
    global frame
    global vertical_power
    global lateral_power
    
    global heading_power
    global lane_lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        pid_x = PID(50, 0, 0, 100)
        pid_y = PID(50, 0, 0, 100)


        while True:
            if video.frame_available():
                # print("\n\n\nFrame found\n\n\n")
                frame = video.frame()
                if predator:
                    try:
                        powers, color_img = process(frame, pid_x, pid_y, at_detector)
                    except:
                        powers = [0, 0]
                    if not powers:
                        continue
                    lateral_power = powers[1]
                    vertical_power = powers[0]
                    print(f'{lateral_power} {vertical_power}')
                else:
                    try:
                        msg = bluerov.recv_match(type="ATTITUDE", blocking=True)
                        yaw = msg.yaw
                        yaw_rate = msg.yawspeed

                        powers = follow_lane(frame, [yaw, yaw_rate], 49, 50, 3, 500, 40)
                        if not powers:
                            powers = [0,0]
                        heading_power, lane_lateral_power = powers


                    except:
                        continue
                
    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power
    while True:
        bluerov.arm()
        bluerov.set_vertical_power(int(vertical_power))
        if predator:
            bluerov.set_lateral_power(int(lateral_power))
        else:
            bluerov.set_lateral_power(int(lane_lateral_power))
            
        bluerov.set_yaw_rate_power(int(heading_power))



# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")
