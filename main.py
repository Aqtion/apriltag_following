from threading import Thread, Event
from time import sleep

from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import numpy as np
from processing import *

# TODO: import your processing functions


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


def _get_frame():
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            if video.frame_available():
                frame = video.frame()
                dimensions = frame.shape
                height = dimensions[0]/2
                width = dimensions[1]/2
                at_detector = Detector(families='tag36h11',
                                nthreads=1,
                                quad_decimate=1.0,
                                quad_sigma=0.0,
                                refine_edges=1,
                                decode_sharpening=0.25,
                                debug=0)
                try:
                    tags = process_frame(frame, at_detector)
                    # img = draw_lines(frame, tags, pid_horizontal, pid_vertical)
                    # plt.imshow(img)
                    # plt.show()
                    lateral_power, vertical_power = pid_output(pid_horizontal, pid_vertical, tags, frame)
                    
                except:
                    pass
                            # TODO: set vertical_power and lateral_power here
    except KeyboardInterrupt:
        return


def _send_rc():
    bluerov.set_vertical_power(vertical_power)
    bluerov.set_lateral_power(lateral_power)


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
