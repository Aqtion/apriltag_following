from pid import PID
import numpy as np

 
def get_to_heading(pid_heading, desired_heading, current_heading, yaw_rate):
    desired_heading_deg = np.rad2deg(desired_heading)

    if desired_heading_deg > np.pi:
        desired_heading_deg = desired_heading_deg - 2*np.pi

    desired_heading = np.deg2rad(desired_heading_deg)

    clockwise_error = (desired_heading - current_heading) % (2 * np.pi)
    
    if (desired_heading <= np.pi and desired_heading >= 0) and (
        current_heading > -np.pi and current_heading < 0
    ):
        first_step = np.pi + current_heading
        second_step = np.pi - desired_heading
        counter_clockwise_error = -(first_step + second_step)
    else:
        counter_clockwise_error = desired_heading - current_heading

    clockwise_deg_error = np.rad2deg(clockwise_error)
    counter_clockwise_deg_error = np.rad2deg(counter_clockwise_error)

    c_e = abs(clockwise_error)
    cc_e = abs(counter_clockwise_error)

    if c_e < cc_e:
        error = clockwise_error
        output = pid_heading.update(error, error_derivative=yaw_rate)
    elif c_e > cc_e:
        error = counter_clockwise_error
        output = pid_heading.update(error, error_derivative=yaw_rate)

    print("Error: ", np.rad2deg(error))

    output = pid_heading.update(error, error_derivative=yaw_rate)
    print("Output: ", output)

    return output


