import cv2
import matplotlib.pyplot as plt
import numpy as np

def get_tags(frame, at_detector):
    dimensions = frame.shape
    height = dimensions[0]/2
    width = dimensions[1]/2
    cameraMatrix = np.array([ 1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3,3))
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params = camera_params, tag_size=0.1)
    color_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    return [tags, color_img]

def draw_center(color_img):
    dimensions = color_img.shape
    height = dimensions[0]/2
    width = dimensions[1]/2
    crosshair_offset = 20
    crosshair_thicc = 2

    cv2.line(color_img, (int(width - crosshair_offset), int(height)), (int(width + crosshair_offset), int(height)), (255,0,0), crosshair_thicc)
    cv2.line(color_img, (int(width), int(height - crosshair_offset)), (int(width), int(height + crosshair_offset)), (255,0,0), crosshair_thicc)

def process(frame, pid_x, pid_y, at_detector):
    tags_img = get_tags(frame, at_detector)
    tags = tags_img[0]
    color_img = tags_img[1]

    draw_center(color_img)

    try:
        data = get_errors(color_img, tags, True)

        # print(data)

        errors = data[0]
        centers = data[1]

        powers = get_powers(errors, pid_x, pid_y)

        draw_powers(color_img, powers)

        draw_tag_center(color_img, centers)

        draw_line_to_center(color_img, centers)

        return powers, color_img
    except:
        pass

def get_errors(color_img, tags, draw):
    x_error = 0
    y_error = 0
    center_x = 0
    center_y = 0

    for tag in tags:
        translation_matrix = tag.pose_t.reshape(1,3)
            
        x = translation_matrix[0][0]
        y = translation_matrix[0][1]
        # z = translation_matrix[0][2]

        x_error += x
        y_error += y
        
        center_x += tag.center[0]
        center_y += tag.center[1]

        if draw:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 5)

    center_x = center_x / len(tags)
    center_y = center_y / len(tags)
    

    avg_x_error = (color_img.shape[0]/2 - center_x) / color_img.shape[0]
    avg_y_error = -1 * (color_img.shape[1]/2 - center_y) / color_img.shape[1]

    return [[avg_x_error, avg_y_error], [center_x, center_y]]

def draw_tag_center(color_img, centers):
    dim = get_dimensions(color_img)
    height = dim[0]/2
    width = dim[1]/2

    center_x = centers[0]
    center_y = centers[1]

    crosshair_offset = 5
    crosshair_thicc = 2

    cv2.line(color_img, (int(center_x-crosshair_offset), int(center_y)), (int(center_x+crosshair_offset), int(center_y)), (0,0,255), crosshair_thicc)
    cv2.line(color_img, (int(center_x), int(center_y-crosshair_offset)), (int(center_x), int(center_y+crosshair_offset)), (0,0,255), crosshair_thicc)


def get_powers(errors, pid_x, pid_y):
    x_error = errors[0]
    y_error = errors[1]
    x_output = pid_x.update(x_error)
    y_output = pid_y.update(y_error)
    return [x_output, y_output]

def draw_powers(color_img, powers):
    dim = get_dimensions(color_img)
    height = dim[0]
    width = dim[1]

    x_output = round(powers[0], 3)
    y_output = round(powers[1], 3)

    str_x_out = "x_output: " + str(x_output)
    str_y_out = "y_output: " + str(y_output)

    x_side_offset = 100
    top_offset = 100

    y_side_offset = 600

    txt_thicc = 2
    cv2.putText(color_img, str_x_out, (int(x_side_offset),int(top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), txt_thicc, cv2.LINE_AA) 
    cv2.putText(color_img, str_y_out, (int(width - y_side_offset),int(top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), txt_thicc, cv2.LINE_AA)

def get_dimensions(color_img):
    return color_img.shape

def draw_line_to_center(color_img, centers):
    dim = get_dimensions(color_img)
    height = dim[0]/2
    width = dim[1]/2

    center_x = centers[0]
    center_y = centers[1]
    cv2.line(color_img, (int(width), int(height)), (int(center_x), int(center_y)), (255,0,0), 5)
