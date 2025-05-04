class PIDController:
    """
    PID Kontroler za poboljšanje upravljanja
    
    Ova klasa implementira proporcionalno-integralno-derivativni (PID) kontroler
    koji se koristi za glađenje i stabilizaciju upravljačkih komandi.
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-float('inf'), float('inf'))):
        """
        Inicijalizacija PID kontrolera
        
        Parametri:
        - kp: Proporcionalni koeficijent (reaguje direktno na trenutnu grešku)
        - ki: Integralni koeficijent (reaguje na akumuliranu grešku tokom vremena)
        - kd: Derivativni koeficijent (reaguje na brzinu promene greške)
        - output_limits: Granice izlazne vrednosti (min, max)
        """
        self.kp = kp  # Proporcionalni faktor
        self.ki = ki  # Integralni faktor
        self.kd = kd  # Derivativni faktor
        
        self.previous_error = 0  # Prethodna greška za izračunavanje derivativnog člana
        self.integral = 0  # Akumulirana greška za izračunavanje integralnog člana
        self.output_limits = output_limits  # Granice izlazne vrednosti
        self.last_output = 0  # Poslednja izračunata vrednost
        
    def compute(self, error, dt=1.0):
        """
        Izračunava PID kontrolnu vrednost na osnovu greške
        
        Parametri:
        - error: Trenutna greška (željena vrednost - trenutna vrednost)
        - dt: Vremenski interval između dva poziva (za skaliranje I i D članova)
        
        Povratna vrednost:
        - Izračunata kontrolna vrednost
        """
        # Izračunavanje proporcionalnog člana
        p_term = self.kp * error
        
        # Izračunavanje integralnog člana sa zaštitom od "windup" efekta
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Izračunavanje derivativnog člana
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        # Izračunavanje ukupnog izlaza
        output = p_term + i_term + d_term
        
        # Ograničavanje izlaza ako su postavljena ograničenja
        if self.output_limits[0] != -float('inf') or self.output_limits[1] != float('inf'):
            output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Čuvanje vrednosti za sledeću iteraciju
        self.previous_error = error
        self.last_output = output
        
        return output
        
    def reset(self):
        """
        Resetuje PID kontroler na početne vrednosti
        Korisno kada se desi nagli prekid ili promena u sistemu
        """
        self.previous_error = 0
        self.integral = 0
        self.last_output = 0
        
    def set_tunings(self, kp=None, ki=None, kd=None):
        """
        Podešava PID koeficijente
        
        Parametri:
        - kp: Novi proporcionalni koeficijent (ako nije None)
        - ki: Novi integralni koeficijent (ako nije None)
        - kd: Novi derivativni koeficijent (ako nije None)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
            
    def set_output_limits(self, min_output, max_output):
        """
        Postavlja granice izlazne vrednosti
        
        Parametri:
        - min_output: Minimalna dozvoljena vrednost izlaza
        - max_output: Maksimalna dozvoljena vrednost izlaza
        """
        self.output_limits = (min_output, max_output)
        
"""
Primer korišćenja:

# Kreiranje PID kontrolera sa odgovarajućim parametrima
# kp=0.5, ki=0.1, kd=0.05, granice izlaza (-30, 30)
pid = PIDController(kp=0.5, ki=0.1, kd=0.05, output_limits=(-30, 30))

# U glavnoj petlji:
while True:
    # Izračunavanje greške (npr. odstupanje od centra trake)
    error = target_position - current_position
    
    # Dobijanje kontrolne vrednosti (npr. ugao volana)
    control_value = pid.compute(error)
    
    # Primena kontrolne vrednosti na aktuator
    set_steering_angle(control_value)
    
    # Ako je potrebno resetovati kontroler (npr. pri promeni režima)
    # pid.reset()
"""