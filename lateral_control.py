from pid import PID


pid = PID(30, 0, -0.5, 100)

def get_to_position(desired_position, current_position):
    error = desired_position - current_position
    output = pid.calculate(error)

    return output


