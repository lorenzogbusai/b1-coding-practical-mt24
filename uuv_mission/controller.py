class Controller:
    def __init__(self, kp, kd, ki=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.previous_error = 0
        self.integral = 0.0

    def set_gains(self, kp, kd, ki=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki

    def compute_action(self, error, dt):
        raise NotImplementedError("This method should be overridden by subclasses")

class PDController(Controller):
    def __init__(self, kp, kd):
        super().__init__(kp, kd)

    def compute_action(self, error, dt):
        derivative = (error - self.previous_error) / dt
        action = self.kp * error + self.kd * derivative
        self.previous_error = error
        return action

class PIDController(Controller):
    def __init__(self, kp, ki, kd):
        super().__init__(kp, kd, ki)

    def compute_action(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        action = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return action
