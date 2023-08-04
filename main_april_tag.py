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
from apriltag_processing import *


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

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)


def _get_frame():
    global frame
    global vertical_power
    global lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        pid_x = PID(30, 0, 0, 100)
        pid_y = PID(25, 0, 0, 100)

        while True:
            if video.frame_available():
                # print("\n\n\nFrame found\n\n\n")

                frame = video.frame()

                try:
                    powers, color_img = process(frame, pid_x, pid_y, at_detector)
                except:
                    _, _ = pid_x.update(0), pid_y.update(0)
                    powers = [0, 0]

                if not powers:
                    continue

                lateral_power, vertical_power = powers

                print(f"{lateral_power} {vertical_power}")

    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power
    bluerov.arm()
    bluerov.mav_connection.set_mode(19)
    while True:
        bluerov.arm()
        # bluerov.set_vertical_power(int(vertical_power))
        # bluerov.set_lateral_power(int(lateral_power))


# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# bluerov.set_rc_channels_to_neutral()
# bluerov.disarm()
# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.set_rc_channels_to_neutral()
    bluerov.disarm()
    print("Exiting...")
