from threading import Thread, Event
from time import sleep
from dt_apriltags import Detector
import numpy as np
from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from lane_following import process_image
import math


# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
pid_horizontal = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Creates the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

yaw_power = 0
lane_lateral_power = 0

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

    global yaw_power
    global lane_lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        lateral_pid = PID(50, 0, 0, 0.9)
        heading_pid = PID(25, 0, 0, 1.3)

        while True:
            if video.frame_available():
                # print("\n\n\nFrame found\n\n\n")
                frame = video.frame()
                height, width, channels = frame.shape

                try:
                    b, m = process_image(frame)
                    delta = math.pi / 2 - (
                        math.atan(m) if (math.atan(m) > 0) else math.atan(m) + math.pi
                    )
                    lateral_offset = (b - width / 2) / width
                except:
                    delta, lateral_offset = 0, 0
                
                print(f"Delta: {delta}, Lateral Offset: {lateral_offset}")

                yaw_power = heading_pid.update(delta)
                lane_lateral_power = lateral_pid.update(lateral_offset)

                # print(f"Heading Output: {yaw_power}, Lateral Output: {lateral_power}")

    except KeyboardInterrupt:
        return


def _send_rc():
    global yaw_power, lateral_power
    bluerov.set_rc_channels_to_neutral()
    while True:
        bluerov.arm()
        bluerov.set_yaw_rate_power(yaw_power)
        bluerov.set_lateral_power(int(lane_lateral_power))


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
