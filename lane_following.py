def get_lane_center(lane):
    line1 = lane[0]
    line2 = lane[1]
    avg_x_intercept = (line1[0] + line2[0]) / 2
    avg_slope = 0
    x1, y1, x2, y2 = line1
    avg_slope += (y2 - y1) / (x2 - x1)
    x1, y1, x2, y2 = line2
    avg_slope += (y2 - y1) / (x2 - x1)
    return avg_x_intercept, avg_slope


def recommend_direction(frame, avg_x_intercept, avg_slope):
    dimensions = frame.shape
    center_cam = dimensions[1]/2

    tol_lr = 100
    dir = ""
    if abs(avg_x_intercept - center_cam) < tol_lr:
        dir = "go forward"
    elif avg_x_intercept > center_cam:
        dir = "go left"
    elif avg_x_intercept < center_cam:
        dir = "go right"
    if avg_slope < 0:
        dir += " + turn left"
    if avg_slope > 0:
        dir += " + turn right"
    return dir
