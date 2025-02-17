import numpy as np

class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, max_integral: float = 100.0):
        self.Kp = Kp  # Proporcionalni faktor
        self.Ki = Ki  # Integralni faktor
        self.Kd = Kd  # Derivativni faktor
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.max_integral = max_integral  # Anti-windup zaštita

    def update(self, error: float, dt: float = 0.1) -> float:
        # Računanje integralnog člana s ograničenjem
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        
        # Računanje derivativnog člana
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # PID izlaz
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        
        # Pamćenje greške za sljedeću iteraciju
        self.previous_error = error
        
        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def set_params(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd