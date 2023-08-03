from lane_detection import (
    detect_lanes,
    detect_lines,
    get_slopes_intercepts,
)

import cv2


def get_lane_center(lanes, x_center, height):
    center_lane = [[x] for x in lanes[0]]

    if len(lanes) > 1:
        for pair in lanes[1:]:
            lane = [[x] for x in pair]

            # Debug, wrong intercept(use bottom instead of top)
            _, intercepts_0 = get_slopes_intercepts(center_lane, height)
            _, intercepts_1 = get_slopes_intercepts(lane, height)

            b_0 = sum(intercepts_0) / len(intercepts_0)
            b_1 = sum(intercepts_1) / len(intercepts_1)

            if abs(b_1 - x_center) < abs(b_0 - x_center):
                center_lane = lane

    slopes, intercepts = get_slopes_intercepts(center_lane, height)
    b = sum(intercepts) / len(intercepts)

    return b, center_lane


def draw_lane_center(img, center_lane, height):
    if len(center_lane) > 1:
        slopes, intercepts = get_slopes_intercepts(center_lane, height)

        slope = sum(slopes) / len(slopes)
        intercept = sum(intercepts) / len(intercepts)

        lane_0, lane_1 = center_lane

        x11, y11, x12, y12 = lane_0[0]

        try:
            m_0 = (y12 - y11) / (x12 - x11)
        except:
            m_0 = 1.01e2

        x21, y21, x22, y22 = lane_1[0]

        try:
            m_1 = (y22 - y21) / (x22 - x21)
        except:
            m_1 = 1e2

        # m1(x - x1) = m2(x-x2)

        x_int1 = 0
        x_int2 = 0

        if y11 > y12:
            x_int1 = x11
        elif y12 > y11:
            x_int1 = x12

        if y21 > y22:
            x_int2 = x21
        elif y22 > y21:
            x_int2 = x22

        try:
            x_intersection = (m_1 * x_int2 - m_0 * x_int1) / (m_1 - m_0)
        except:
            x_intersection = (x_int1 + x_int2) / 2

        y_intersection = m_0 * (x_intersection - x_int1) + height

        m_final = (height - y_intersection) / (intercept - x_intersection)

        # img = cv2.line(
        #     img,
        #     (int(intercept), height),
        #     (int(x_intersection), int(y_intersection)),
        #     (255, 0, 0),
        #     4,
        # )

    else:
        x1, y1, x2, y2 = center_lane[0][0]

        m_final = (y2 - y1) / (x2 - x1)

        # img = cv2.line(
        #     img,
        #     (int(x1), int(y1)),
        #     (int(x2), int(y2)),
        #     (255, 0, 0),
        #     4,
        # )

    return m_final


def recommend_strafe_direction(center, slope, width):
    x_center = width / 2
    x_tol = width / 10
    m_tol = 3e1

    difference = center - x_center
    strafe_direction = ""
    turn_direction = ""

    if abs(difference) < x_tol:
        strafe_direction = "forward"
    elif difference > x_tol:
        strafe_direction = "right"
    else:
        strafe_direction = "left"

    if abs(slope) > m_tol:
        turn_direction = "forward"
    elif slope < 0:
        turn_direction = "left"
    else:
        turn_direction = "right"

    return (strafe_direction, turn_direction)


def process_image(img):
    height, width, channels = img.shape

    # lines = detect_lines(img, 30, 50, 3, 50, 30)  # (land)
    lines = detect_lines(img, 5, 70, 3, 50, 16)  # (underwater)

    if len(lines) < 1:
        return width / 2, 1e4, img

    lanes = detect_lanes(lines, width, height)

    if len(lanes) < 1:
        return width / 2, 1e4, img

    b, center_lane = get_lane_center(lanes, width / 2, height)

    m_final = draw_lane_center(img, center_lane, height)

    # return b, m, img

    return b, m_final
