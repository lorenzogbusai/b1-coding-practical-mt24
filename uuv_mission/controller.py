class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0

    def set_gains(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def compute_action(self, error, dt):
        derivative = (error - self.previous_error) / dt
        action = self.kp * error + self.kd * derivative
        self.previous_error = error
        return action
