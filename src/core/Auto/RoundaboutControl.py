import cv2
import numpy as np
import time
from src.ImageProcessing.LaneDetect.pid_controller import PIDController
from typing import Tuple


class RoundaboutController:
    def __init__(self, width: int, height: int, logging: bool = False, debugging: bool = False):

        self.width = width
        self.height = height
        self.logging = logging
        self.debugging = debugging
        
        # Stanje kontrolera
        self.active = False                 # Da li je kontroler aktivan
        self.target_exit = None             # Broj izlaza na koji treba izaći
        self.current_phase = None           # Trenutna faza (entry, follow_right, follow_left, exit)
        self.exit_count = 0                 # Broj izlaza koje smo prošli
        self.phase_start_time = None        # Vrijeme početka trenutne faze
        
        # PID kontroleri za praćenje lijeve i desne linije
        self.right_pid = PIDController(kp=0.4, ki=0.01, kd=0.0, kaw = 0, output_limits=(-25, 25))
        self.left_pid = PIDController(kp=0.4, ki=0.01, kd=0.0, kaw = 0, output_limits=(-25, 25))
        
        # Vremena trajanja faza (u sekundama)
        self.entry_phase_time = 5        # Vrijeme za fazu ulaska
        self.exit_phase_time = 8.5       # Vrijeme za fazu izlaska
        
        # Parametri za detekciju izlaza
        self.exit_detection_region = {      # Region u kojem se detektuje izlaz
            'x_min': int(width * 0.6),      # 60% širine slike (desno)
            'y_min': int(height * 0.5),     # 50% visine (sredina)
            'x_max': width,                 # Desna ivica slike
            'y_max': height                 # Donja ivica slike
        }
        
        # Potrebna udaljenost od linije
        self.right_line_target_offset = 130  # Željena udaljenost od desne linije (piksel)
        self.left_line_target_offset = 155 # Željena udaljenost od lijeve linije (piksel)
        
        # Podatci o zadnjem detektovanom izlazu
        self.last_exit_data = None  # sada će ovo biti (x1, y1, x2, y2) ili None
        self.last_exit_detected = False
        
        # Dodavanje varijable za praćenje vremena za izračun dt
        self.last_process_time = time.time()
    
    def start(self, command: str):

        if not command.startswith("Exit "):
            if self.debugging:
                print("Greška: Komanda mora biti u formatu 'Exit X'")
            return False

        try:
            target_exit = int(command.split(" ")[1])
        except (IndexError, ValueError):
            if self.debugging:
                print("Greška: Nije moguće parsirati broj izlaza iz komande")
            return False

        if target_exit < 1:
            if self.debugging:
                print("Greška: Broj izlaza mora biti >= 1")
            return False

        self.target_exit = target_exit
        self.active = True
        self.current_phase = "entry"
        self.exit_count = 0
        self.phase_start_time = time.time()
        self.right_pid.reset()
        self.left_pid.reset()

        if self.debugging:
            print(f"RoundaboutController: Započeta kontrola, cilj izlaz {target_exit}")

        return True

    
    def stop(self):
        """Zaustavlja kontrolu vožnje kroz kružni tok."""
        self.active = False
        self.current_phase = None
        
        if self.debugging:
            print("RoundaboutController: Kontrola zaustavljena")
    
    def is_exit_in_region(self, exit_data):
        if exit_data is None:
            return False
        return True
    
    def process_frame(self, left_x, right_x, exit_data, leftVisible, rightVisible) -> Tuple[int, bool]:
        if not self.active:
            return 0, False
        
        
        # Izračun dt - vreme proteklo od poslednjeg poziva ove funkcije
        current_time = time.time()
        dt = current_time - self.last_process_time
        self.last_process_time = current_time
        
        # Trenutno vrijeme
        phase_elapsed = current_time - self.phase_start_time
        
        # Praćenje izlaza
        self._track_exit(exit_data)
        
        # Upravljanje fazama
        if self.current_phase == "entry":
            # Faza ulaska - pratimo desnu liniju određeno vrijeme
            if phase_elapsed >= self.entry_phase_time and leftVisible:
                # Prelazak na fazu praćenja lijeve linije
                self.current_phase = "follow_left"
                self.phase_start_time = current_time
                if self.debugging:
                    print("RoundaboutController: Prelazak na 'follow_left'")
            
            x = int(self._follow_right_line(right_x, rightVisible, dt)*10)
            if x < 0:
                x = 6
            return x, False
            
        elif self.current_phase == "follow_left":
            # Faza praćenja lijeve linije - sve dok ne dođemo do ciljanog izlaza
            if self.exit_count >= self.target_exit and rightVisible:
                # Pronađen ciljani izlaz, prelazak na fazu izlaska
                self.current_phase = "exit"
                self.phase_start_time = current_time
                if self.debugging:
                    print(f"RoundaboutController: Dostignut ciljni izlaz ({self.exit_count}), prelazak na 'exit'")
            
            return int(self._follow_left_line(left_x,leftVisible, dt)*10), False
            
        elif self.current_phase == "exit":
            # Faza izlaska - pratimo desnu liniju određeno vrijeme
            if phase_elapsed >= self.exit_phase_time:
                # Završetak kontrole kružnog toka
                self.stop()
                if self.debugging:
                    print("RoundaboutController: Kontrola završena uspješno")
                return 0.0, True

            return int(self._follow_right_line(right_x,rightVisible, dt)*10), False
            
        return 0, False
    
    def _track_exit(self, exit_data):
        # Proveri da li je izlaz prisutan i u regionu
        exit_detected = self.is_exit_in_region(exit_data)
        # Brojanje kada izlaz nestane iz regiona (tranzicija sa detected na not detected)
        if not self.last_exit_detected and exit_detected:
            self.exit_count += 1
            if self.debugging:
                print(f"RoundaboutController: Detektovan izlaz #{self.exit_count}")
        
        self.last_exit_data = exit_data  # čuvamo podatke o trenutnom izlazu
        self.last_exit_detected = exit_detected
    
    def _follow_right_line(self, right_x,rightVisible, dt):
        
        center_x = self.width *0.47
        target_position = right_x - self.right_line_target_offset
        error = center_x - target_position

        steering_angle = -self.right_pid.compute(error, dt)

        if self.debugging:
            print(f"Follow right: error={error:.1f}, angle={steering_angle:.1f}")
            
        return steering_angle
    
    def _follow_left_line(self, left_x,leftVisible, dt):
        if not leftVisible:
            target_position = left_x + self.left_line_target_offset - 150
        else:
            target_position = left_x + self.left_line_target_offset
        
        # Računanje greške (odstupanje od željene udaljenosti)
        center_x = self.width *0.47
        error = center_x - target_position
        
        # PID kontrola
        steering_angle = -self.left_pid.compute(error, dt)

        if steering_angle > 10:
            steering_angle = 10
        if self.debugging:
            print(f"Follow left: error={error:.1f}, angle={steering_angle:.1f}")
            
        return steering_angle