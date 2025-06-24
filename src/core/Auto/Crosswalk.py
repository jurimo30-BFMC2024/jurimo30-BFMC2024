# filepath: /home/bagi/Desktop/BOS/jurimo30-BFMC2024/src/core/Auto/croswalk.py

import time

class CrosswalkController:
    def __init__(self):
        self.finished = False
        self.waiting_time = time.time()
        self.previous_side = None
        self.detected_other_side = False
        self.waiting_complete = False
        self.started = False  # Flag to track if we've started detection
        print("[Crosswalk] Controller initialized")

    def determine_side(self, stephanie_position):
        x1, y1, x2, y2 = stephanie_position
        x = (x1 + x2) / 2

        # print(f"x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2} x: {x}")

        if x < 200:
            return "left"
        elif x > 312:
            return "right"
        else:
            return "middle"

    def control(self, stephanie_position, forward_enabled=True):
        # First detection of Stephanie - set started flag if not already set
        if  not self.started:
            self.started = True
            self.waiting_time = time.time()
            print("[Crosswalk] Strarted")
            if stephanie_position is None:
                print("[Crosswalk] No Stephanie detected")
        
        if (stephanie_position is None and self.previous_side is None) or self.waiting_complete:
            self.curentTime = time.time()
            if self.curentTime - self.waiting_time > 1.2 or not forward_enabled:
                print("[Crosswalk] Waiting complete")
                self.finished = False
                self.waiting_time = time.time()
                self.previous_side = None
                self.detected_other_side = False
                self.waiting_complete = False
                self.started = False  # Flag to track if we've started detection
                return 0, 400, True
            else:
                return 0, 400, False
        
        if stephanie_position is not None and self.previous_side is None:
            # First detection of Stephanie
            self.previous_side = self.determine_side(stephanie_position)
            self.waiting_complete = False
            print(f"[Crosswalk] Detected Stephanie on the {self.previous_side} side")
        
        if stephanie_position is not None and self.previous_side is not None and not self.waiting_complete:
            # Check if Stephanie is on the other side
            current_side = self.determine_side(stephanie_position)
            
            if current_side != self.previous_side and current_side != "middle":
                self.detected_other_side = True
                self.waiting_complete = True

                self.waiting_time = time.time()
                print("[Crosswalk] Detected Stephanie on the other side")
            else:
                print(f"[Crosswalk] Stephanie is still on the {current_side} side")
        
        return 0, 0, False

