from pid import PID
import numpy as np


def get_to_heading(pid_heading, desired_heading, current_heading, yaw_rate):
    yaw = yaw % (2 * np.pi)

    # calculate error
    error = desired_heading - yaw

    # Implementation by mapping to sine
    error = error % (2 * np.pi) - np.pi

    if error > np.pi / 2:
        error = 1
    elif error < -np.pi / 2:
        error = -1
    else:
        error = np.sin(error)

    output = pid_heading.update(error, error_derivative=yaw_rate)

    print("Error: ", np.rad2deg(error))
    print("Output: ", output)

    return output
