from pid import *
from lane_detection import *
from lane_following import *
from heading_control import *
from lateral_control import *

import math


def follow_lane(frame, data):
    # plt.imshow(frame)

    _, width, _ = frame.shape

    b, m = process_image(frame)

    desired_heading = math.atan(m)

    current_heading, yaw_rate = data

    heading_power = get_to_heading(desired_heading, current_heading, yaw_rate)
    lateral_power = get_to_position(b, width / 2)

    return [heading_power, lateral_power]
