from pid import *
from lane_detection import *
from lane_following import *
from heading_control import *
from lateral_control import *

import math


def follow_lane(frame, data, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100, maxLineGap=10):
    plt.imshow(frame)

    dimensions = frame.shape

    height = dimensions[0]

    width = dimensions[1]

    lines = detect_lines(frame, threshold1, threshold2, apertureSize, minLineLength, maxLineGap)
    if not lines:
        return
    print("lines: ", lines)
    lanes = detect_lanes(lines)

    draw_lines(frame, lines)
    draw_lanes(frame, lanes)

    print("lanes: ", lanes)

    center_lane = get_center_lane(lanes)

    print("center lane: ", center_lane)

    center_of_lane = get_lane_center(center_lane)
    x_int, slope = center_of_lane

    desired_heading = math.atan(slope)
    desired_pos = x_int

    current_heading, yaw_rate = data


    heading_power = get_to_heading(desired_heading, current_heading, yaw_rate)
    lateral_power = get_to_position(desired_pos, width/2)

    print(recommend_direction(center_of_lane))

    return [heading_power, lateral_power]
