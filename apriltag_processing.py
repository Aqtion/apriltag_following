import cv2
import matplotlib.pyplot as plt
import numpy as np
import heading_control
from scipy.spatial.transform import Rotation as R


def get_tags(frame, at_detector):
    cameraMatrix = np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3, 3))
    camera_params = (
        cameraMatrix[0, 0],
        cameraMatrix[1, 1],
        cameraMatrix[0, 2],
        cameraMatrix[1, 2],
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1
    )

    color_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    return [tags, color_img]


def draw_center(color_img):
    height, width, _ = color_img.shape
    height, width = map(lambda i: i / 2, (height, width))

    crosshair_offset = 20

    cv2.line(
        color_img,
        (int(width - crosshair_offset), int(height)),
        (int(width + crosshair_offset), int(height)),
        (255, 0, 0),
        5,
    )

    cv2.line(
        color_img,
        (int(width), int(height - crosshair_offset)),
        (int(width), int(height + crosshair_offset)),
        (255, 0, 0),
        5,
    )


def process(frame, pid_x, pid_y, pid_heading, pid_z, at_detector, yaw, yaw_rate):
    tags_img = get_tags(frame, at_detector)
    tags, color_img = tags_img

    draw_center(color_img)

    try:
        data = get_errors(color_img, tags, True)

        errors, centers = data

        powers = get_powers(errors, pid_x, pid_y, pid_z, pid_heading, yaw, yaw_rate)

        draw_powers(color_img, powers)

        draw_tag_center(color_img, centers)

        draw_line_to_center(color_img, centers)

        return powers, color_img
    except:
        pass


def get_errors(color_img, tags, draw):
    x_error, y_error, z_error = 0, 0, 0

    center_x, center_y = 0, 0

    theta_error = 0

    for tag in tags:
        translation_matrix = tag.pose_t.reshape(1, 3)
        rotation_matrix = tag.pose_R
        rotations = R.from_matrix(rotation_matrix)

        x, y, z = translation_matrix[0]

        theta = R.from_euler("zyx", rotations, degrees=True)[1]

        theta_error += theta

        x_error += x
        y_error += y
        z_error += z

        center_x += tag.center[0]
        center_y += tag.center[1]

        if draw:
            for idx in range(len(tag.corners)):
                cv2.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),
                    5,
                )

    center_x = center_x / len(tags)
    center_y = center_y / len(tags)

    z_error = z_error / len(tags)

    avg_x_error = (color_img.shape[0] / 2 - center_x) / color_img.shape[0]
    avg_y_error = -1 * (color_img.shape[1] / 2 - center_y) / color_img.shape[1]
    avg_theta_error = theta_error / len(tags)

    return [[avg_x_error, avg_y_error, z_error, avg_theta_error], [center_x, center_y]]


def draw_tag_center(color_img, centers):
    center_x, center_y = centers

    crosshair_offset = 5

    cv2.line(
        color_img,
        (int(center_x - crosshair_offset), int(center_y)),
        (int(center_x + crosshair_offset), int(center_y)),
        (0, 0, 255),
        5,
    )

    cv2.line(
        color_img,
        (int(center_x), int(center_y - crosshair_offset)),
        (int(center_x), int(center_y + crosshair_offset)),
        (0, 0, 255),
        5,
    )


def get_powers(errors, pid_x, pid_y, pid_z, pid_heading, yaw, yaw_rate):
    x_error, y_error, z_error, heading_error = errors[3]

    x_output = pid_x.update(x_error)
    y_output = pid_y.update(y_error)
    z_output = pid_z.update(z_error)

    heading_output = heading_control.get_to_heading(
        pid_heading, yaw + heading_error, yaw, yaw_rate
    )

    # Currently uses the April Tag poses
    return [x_output, y_output, z_output, heading_output]


def draw_powers(color_img, powers):
    height, width, _ = color_img.shape

    x_output, y_output, z_output, heading_output = powers

    str_x_out = "x_output: " + str(x_output)
    str_y_out = "y_output: " + str(y_output)
    str_z_out = "z_output: " + str(z_output)
    str_heading_out = "heading_output: " + str(heading_output)

    x_side_offset, top_offset, y_side_offset = 100, 100, 100

    text_thickness = 2

    cv2.putText(
        color_img,
        str_x_out,
        (int(x_side_offset), int(top_offset)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        text_thickness,
        cv2.LINE_AA,
    )

    cv2.putText(
        color_img,
        str_y_out,
        (int(width - y_side_offset), int(top_offset)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        text_thickness,
        cv2.LINE_AA,
    )

    cv2.putText(
        color_img,
        str_z_out,
        (int(x_side_offset), int(height - top_offset)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        text_thickness,
        cv2.LINE_AA,
    )

    cv2.putText(
        color_img,
        str_heading_out,
        (int(width - x_side_offset), int(height - top_offset)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        text_thickness,
        cv2.LINE_AA,
    )


def draw_line_to_center(color_img, centers):
    height, width, _ = color_img.shape
    height, width = map(lambda i: i / 2, (height, width))

    center_x, center_y = centers

    cv2.line(
        color_img,
        (int(width), int(height)),
        (int(center_x), int(center_y)),
        (255, 0, 0),
        5,
    )
