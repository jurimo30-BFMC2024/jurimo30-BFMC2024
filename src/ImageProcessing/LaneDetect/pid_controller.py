class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, kaw=8, output_limits=(-float('inf'), float('inf'))):
        self.kp = kp  # Proporcionalni faktor
        self.ki = ki  # Integralni faktor
        self.kd = kd  # Derivativni faktor
        # Anti-windup koeficijent; po defaultu jednak ki ako nije specificiran
        self.kaw = kaw

        self.previous_error = 0.0      # Prethodna greška
        self.integral = 0.0            # Integralna akumulacija
        self.output_limits = output_limits  # Granice izlaza
        self.last_output = 0.0         # Poslednji izračunati izlaz

    def compute(self, error, dt=1.0):
        # Proporcionalni član
        p_term = self.kp * error

        # Derivativni član
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative

        # "Idealni" PID izlaz pre saturacije
        u_ideal = p_term + self.integral + self.ki * error * dt + d_term

        # Saturacija izlaza u definisane granice
        u_sat = max(self.output_limits[0], min(self.output_limits[1], u_ideal))

        # Anti-windup korekcija integratora (back-calculation)
        # dI/dt = ki*error + kaw*(u_sat - u_ideal)
        self.integral += (self.ki * error + self.kaw * (u_sat - u_ideal)) * dt

        # Čuvanje za sledeću iteraciju
        self.previous_error = error
        self.last_output = u_sat

        return u_sat

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0

    def set_tunings(self, kp=None, ki=None, kd=None, kaw=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if kaw is not None:
            self.kaw = kaw

    def set_output_limits(self, min_output, max_output):
        self.output_limits = (min_output, max_output)
