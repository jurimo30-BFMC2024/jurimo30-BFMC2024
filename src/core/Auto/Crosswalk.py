import time
from typing import Tuple, Optional

class CrosswalkController:
    # Constants
    LEFT_THRESHOLD = 200
    RIGHT_THRESHOLD = 312
    GO_FORWARD_DURATION = 1.2
    FORWARD_SPEED = 400

    def __init__(self) -> None:
        self.waiting_time = time.time()
        self.previous_side = None
        self.waiting_complete = False
        self.started = False
        print("[Crosswalk] Controller initialized")

    def determine_side(self, stephanie_position: Tuple[float, float, float, float]) -> str:
        """Determine which side Stephanie is on based on position."""
        x1, _, x2, _ = stephanie_position
        center_x = (x1 + x2) / 2

        if center_x < self.LEFT_THRESHOLD:
            return "left"
        elif center_x > self.RIGHT_THRESHOLD:
            return "right"
        return "middle"

    def _go_forward_after_crossing(self) -> Tuple[float, float, bool]:
        """Handle the state when waiting is complete or no detection."""
        current_time = time.time()
        if current_time - self.waiting_time > self.GO_FORWARD_DURATION:
            self._reset_state()
            print("[Crosswalk] Complete, went forward")
            return 0, self.FORWARD_SPEED, True
        return 0, self.FORWARD_SPEED, False

    def _reset_state(self) -> None:
        """Reset all state variables."""
        self.waiting_time = time.time()
        self.previous_side = None
        self.waiting_complete = False
        self.started = False

    def control(self, stephanie_position: Optional[Tuple[float, float, float, float]], forward_enabled: bool = True) -> Tuple[float, float, bool]:
        """Main control loop for crosswalk behavior."""
        if not self.started:
            self._initialize_detection(stephanie_position)
        
        if (stephanie_position is None and self.previous_side is None) or self.waiting_complete:
            if forward_enabled:
                return self._go_forward_after_crossing()    # Waiting completed, go forward
            else:
                self._reset_state()
                print("[Crosswalk] Complete, but forward motion not enabled")
                return 0, 0, True  # Fixed: return 0 speed when forward_enabled is False

        if stephanie_position is not None:
            if self.previous_side is None:
                self._handle_first_detection(stephanie_position)
            elif not self.waiting_complete:
                self._handle_crossing_detection(stephanie_position)

        return 0, 0, False

    def _initialize_detection(self, stephanie_position: Optional[Tuple[float, float, float, float]]) -> None:
        """Initialize the detection process."""
        self.started = True
        self.waiting_time = time.time()
        print("[Crosswalk] Started")
        if stephanie_position is None:
            print("[Crosswalk] No Stephanie detected")

    def _handle_first_detection(self, stephanie_position: Tuple[float, float, float, float]) -> None:
        """Handle the first detection of Stephanie."""
        self.previous_side = self.determine_side(stephanie_position)
        self.waiting_complete = False
        print(f"[Crosswalk] Detected Stephanie on the {self.previous_side} side")

    def _handle_crossing_detection(self, stephanie_position: Tuple[float, float, float, float]) -> None:
        """Handle detection during crossing."""
        current_side = self.determine_side(stephanie_position)
        if current_side != self.previous_side and current_side != "middle":
            self.waiting_complete = True
            self.waiting_time = time.time()
            print("[Crosswalk] Detected Stephanie on the other side")
        else:
            print(f"[Crosswalk] Stephanie is still on the {current_side} side")

