import cv2
import numpy as np
import matplotlib.pyplot as plt


def detect_lines(
    img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100, maxLineGap=10
):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize)  # detect edges
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=180,
        minLineLength=minLineLength,
        maxLineGap=maxLineGap,
    )
    return lines


def draw_lines(img, lines, color=(0, 255, 0)):
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), color, 6)
    return img


def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    for line in lines:
        res = 2160  # resolution
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)
        x_intercept = (res - y2) / slope + x2
        slopes.append(slope)
        intercepts.append(x_intercept)
    return slopes, intercepts


def detect_lanes(lines):
    res = 2160
    slopes, intercepts = get_slopes_intercepts(lines)

    lanes = []
    for i in range(len(slopes)):
        for j in range(i + 1, len(slopes)):
            min_intercept = min(intercepts[i], intercepts[j])
            max_intercept = max(intercepts[i], intercepts[j])
            min_slope = min(slopes[i], slopes[j])
            max_slope = max(slopes[i], slopes[j])
            intercept_ratio = abs(min_intercept / max_intercept)
            slope_ratio = abs(min_slope / max_slope)
            slope_difference = abs(1 / slopes[i] - 1 / slopes[j])

            # print(f"dist1:{abs(intercepts[i]-intercepts[j])}")
            # print(f"slope1:{abs(1/ slopes[i]-1 /slopes[j])}")
            # print(f"dist2:{max_intercept - min_intercept }")
            # print(f"slope2:{slope_difference}")

            if (
                max_intercept - min_intercept > 100
                and max_intercept - min_intercept < 10000
                and slope_difference < 1
            ):
                # m1(x-x1) = m2(x - x2)
                # m1x - m1x1 = m2x - m2x2
                # m1x - m2x = m1x1 - m2x2
                # x = (m1x1 - m2x2) / (m1-m2)
                x = (slopes[i] * intercepts[i] - slopes[j] * intercepts[j]) / (
                    slopes[i] - slopes[j]
                )
                # plug in x into any equation, add resolution to account for cam
                y = slopes[i] * (x - intercepts[i]) + res

                line1 = [intercepts[i], res, x, y]
                line2 = [intercepts[j], res, x, y]
                lanes.append([line1, line2])
    return lanes


def draw_lanes(img, lanes, color=(255, 0, 0)):
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
    cnt = 0
    for lane in lanes:
        color = colors[cnt]
        for line in lane:
            # first point is x-intercept and border of screen (resolution)
            # second point is intersection point of two lines forming the lane
            print("___")
            print(lane)
            x1, y1, x2, y2 = line
            cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 5)

    return img


def draw_lane(img, lane, color=(255, 0, 0)):
    for line in lane:
        x1, y1, x2, y2 = line
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 5)
    return img


def get_center_lane(lanes):
    int_width = -1
    center_lane = ""
    # the closest lane to center of screen will be the "widest lane" relative to the camera
    for lane in lanes:
        line1 = lane[0]
        line2 = lane[1]
        x_int1 = line1[0]
        x_int2 = line2[0]
        int_diff = abs(x_int2 - x_int1)
        if int_width < int_diff:
            int_width = int_diff
            center_lane = lane
    return center_lane
