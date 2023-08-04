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

output_file = 'output_video.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_video = cv2.VideoWriter(output_file, fourcc, 30, (640, 360))

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
longitudinal_power = 0

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
    global longitudinal_power
    
    global heading_power
    global lane_lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        pid_x = PID(60, 0, 0, 100)
        pid_y = PID(50, 0, 0, 100)

        # TODO tune the hell out of this
        pid_z = PID(0.5, 0, -0.005, 100)

        pid_heading = PID(30, 0, -0.5, 100)


        while True:
            if video.frame_available():
                # print("\n\n\nFrame found\n\n\n")
                frame = video.frame()
                # cv2.imwrite("frame.png", frame)
                if predator:
                    try:
                        msg = bluerov.mav_connection.recv_match(type="ATTITUDE", blocking=True)
                        yaw = msg.yaw
                        yaw_rate = msg.yawspeed

                        #print(yaw_rate)

                        # print("Heading: ", np.rad2deg(yaw))

                        powers, color_img = process(frame, pid_x, pid_y, pid_heading, pid_z, at_detector, yaw, yaw_rate)
                        
                        output_video.write(color_img)
                    except:
                        powers = [0, 0, 0, 0]
                    if not powers:
                        continue
                   
                    vertical_power = powers[0]
                    lateral_power = powers[1]
                    longitudinal_power = powers[2]
                    heading_power = powers[3]
                    
                    # if not heading_power  == 0:
                    print(f"heading_power: {heading_power}")
                    # print(f'{lateral_power} {vertical_power} {longitudinal_power} {heading_power}')
                # else:
                #     try:
                #         msg = bluerov.recv_match(type="ATTITUDE", blocking=True)
                #         yaw = msg.yaw 
                #         yaw_rate = msg.yawspeed

                #         powers = follow_lane(frame, [yaw, yaw_rate], 49, 50, 3, 500, 40)
                #         if not powers:
                #             powers = [0,0]
                #         heading_power, lane_lateral_power = powers
                #         print(powers)


                #     except:
                #         continue
                
    except KeyboardInterrupt:
        output_video.release()
        return


def _send_rc():
    global vertical_power, lateral_power, longitudinal_power, heading_power
    while True:
        # bluerov.arm()
        # bluerov.set_vertical_power(int(vertical_power))
        if predator:
            pass
            # bluerov.set_lateral_power(int(lateral_power))
            # bluerov.set_longitudinal_power(int(longitudinal_power))
            # bluerov.set_yaw_rate_power(int(heading_power))
        else:
            pass
            # bluerov.set_lateral_power(int(lane_lateral_power))
            
        



# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

bluerov.set_rc_channel(9, 1600)

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")
