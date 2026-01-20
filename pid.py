class PID():

    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0

    def update(self, error, dt):
        # Derivative term
        derivative = (error - self.prev_error) / dt

        # PD output
        output = self.kp * error + self.kd * derivative

        # Save error for next loop
        self.prev_error = error

        return output

    def reset(self):
        self.prev_error = 0
