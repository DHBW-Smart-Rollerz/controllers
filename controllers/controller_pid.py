class PIDController:
    def __init__(self, kp, ki, kd):
        """
        Initialize the PID controller with given gains and setpoint.
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param setpoint: Desired target value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, target_value):
        """
        Calculate the control output based on the current value and delta_time.
        :param current_value: The current measurement of the process variable
        :return: Control output
        """
        error = target_value - current_value
        delta_time = 0.14
        delta_error = error - self.previous_error

        self.integral += error * delta_time
        derivative = delta_error / delta_time if delta_time > 0 else 0

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error

        return output
