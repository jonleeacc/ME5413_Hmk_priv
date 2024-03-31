# pid.py

class PID:
    def __init__(self, dt, max_value, min_value, Kp, Kd, Ki):
        self.dt = dt
        self.max_value = max_value
        self.min_value = min_value
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.pre_error = 0
        self.integral = 0

    def update_settings(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def calculate(self, setpoint, pv):
        # Calculate error
        error = setpoint - pv

        # Proportional term
        P_term = self.Kp * error

        # Integral term
        self.integral += error * self.dt
        I_term = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.pre_error) / self.dt
        D_term = self.Kd * derivative

        # Calculate total output
        output = P_term + I_term + D_term

        # Restrict to max/min
        output = min(output, self.max_value)
        output = max(output, self.min_value)

        # Save error to previous error
        self.pre_error = error

        return output
