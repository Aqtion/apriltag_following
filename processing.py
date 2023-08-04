import cv2
import matplotlib.pyplot as plt
import numpy as np
import heading_control
from scipy.spatial.transform import Rotation as R

def get_tags(frame, at_detector):
    dimensions = frame.shape
    height = dimensions[0]/2
    width = dimensions[1]/2
    cameraMatrix = np.array([ 353.571428571, 0, 320, 0, 353.571428571, 180, 0, 0, 1]).reshape((3,3))    
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
    cv2.line(color_img, (int(width - crosshair_offset), int(height)), (int(width + crosshair_offset), int(height)), (255,0,0), 5)
    cv2.line(color_img, (int(width), int(height - crosshair_offset)), (int(width), int(height + crosshair_offset)), (255,0,0), 5)

def process(frame, pid_x, pid_y, pid_heading, pid_z, at_detector, yaw, yaw_rate):
    tags_img = get_tags(frame, at_detector)
    tags = tags_img[0]
    color_img = tags_img[1]

    draw_center(color_img)

    try:
        data = get_errors(color_img, tags, True)
        # print(data)
        errors = data[0]
        centers = data[1]

        powers = get_powers(errors, pid_x, pid_y, pid_z, pid_heading, yaw, yaw_rate)

        draw_powers(color_img, powers)

        draw_tag_center(color_img, centers)

        draw_line_to_center(color_img, centers)

        return powers, color_img
    except:
        pass

def get_errors(color_img, tags, draw):
    z_error = 0

    center_x = 0
    center_y = 0
    theta_error = 0

    for tag in tags:
        translation_matrix = tag.pose_t.reshape(1,3)
 
        # print(translation_matrix)

        rotation_matrix = tag.pose_R
        rotations = R.from_matrix(rotation_matrix)

        # print(rotations.as_quat())
        
        # print(rotations)
            
        # x = translation_matrix[0][0]
        # y = translation_matrix[0][1]
        z = translation_matrix[0][2]

        # print(z)

        euler = rotations.as_euler('zyx', degrees=False)

        theta = euler[1]

        theta_error += theta

        z_error += z

        # print("theta error", theta_error)
        # print("z error", z_error)
        
        center_x += tag.center[0]
        center_y += tag.center[1]

        if draw:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 5)

    center_x = center_x / len(tags)
    center_y = center_y / len(tags)

    z_error = z_error / len(tags)
    

    avg_x_error = (color_img.shape[0]/2 - center_y) / color_img.shape[0]
    avg_y_error = -1 * (color_img.shape[1]/2 - center_x) / color_img.shape[1]
    avg_theta_error = theta_error / len(tags)

    #print(avg_theta_error)
    #print(z_error, avg_theta_error)

    return [[avg_x_error, avg_y_error, z_error, avg_theta_error], [center_x, center_y]]

def draw_tag_center(color_img, centers):
    center_x = centers[0]
    center_y = centers[1]

    crosshair_offset = 5

    cv2.line(color_img, (int(center_x-crosshair_offset), int(center_y)), (int(center_x+crosshair_offset), int(center_y)), (0,0,255), 5)
    cv2.line(color_img, (int(center_x), int(center_y-crosshair_offset)), (int(center_x), int(center_y+crosshair_offset)), (0,0,255), 5)


def get_powers(errors, pid_x, pid_y, pid_z, pid_heading, yaw, yaw_rate):
    longitudinal_offset = 0.2
    x_error = errors[0]
    y_error = errors[1]
    z_error = errors[2] - longitudinal_offset
    # print("z_error", z_error)
    heading_error = errors[3] 

    x_output = np.clip(pid_x.update(x_error), -100, 100)
    y_output = np.clip(pid_y.update(y_error), -100, 100)
    z_output = np.clip(pid_z.update(z_error), -100, 100)

    # print(x_output, y_output, z_output)
    heading_output = heading_control.get_to_heading(pid_heading, yaw + heading_error, yaw, yaw_rate)

    return [x_output, y_output, z_output, heading_output]

def draw_powers(color_img, powers):
    dim = get_dimensions(color_img)
    height = dim[0]
    width = dim[1]

    x_output = powers[0]
    y_output = powers[1]
    z_output = powers[2]
    heading_output = powers[3]

    str_x_out = "x_output: " + str(round(x_output, 3))
    str_y_out = "y_output: " + str(round(y_output, 3))
    str_z_out = "z_output: " + str(round(z_output, 3))
    str_heading_out = "heading_output: " + str(round(heading_output, 3))

    side_offset = 25
    top_offset = 25

    smth = 300

    text_thickness = 1

    cv2.putText(color_img, str_x_out, (int(side_offset),int(top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), text_thickness, cv2.LINE_AA) 
    cv2.putText(color_img, str_y_out, (int(width - side_offset - smth),int(top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), text_thickness, cv2.LINE_AA)
    cv2.putText(color_img, str_z_out, (int(side_offset),int(height - top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), text_thickness, cv2.LINE_AA)
    cv2.putText(color_img, str_heading_out, (int(width - side_offset - smth),int(height - top_offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), text_thickness, cv2.LINE_AA)

def get_dimensions(color_img):
    return color_img.shape

def draw_line_to_center(color_img, centers):
    dim = get_dimensions(color_img)
    height = dim[0]/2
    width = dim[1]/2

    center_x = centers[0]
    center_y = centers[1]
    cv2.line(color_img, (int(width), int(height)), (int(center_x), int(center_y)), (255,0,0), 5)
