import cv2
import numpy as np

class ImagePreProcessing:
    def __init__(self, width: int, height: int,logging, debugging=False, pc = False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.l245ogging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc
    
    def gamma_correction(self, image: np.ndarray, gamma: float) -> np.ndarray:
        image_normalized = image  / 255.0
        corrected = np.power(image_normalized, gamma)
        corrected = np.uint8(corrected * 255)
        return corrected
    
    def process_frame(self, frame: np.ndarray):
        processingFrame = frame.copy()
        gray = cv2.cvtColor(processingFrame, cv2.COLOR_RGB2GRAY)
        gama = self.gamma_correction(gray, 12)
        blurred = cv2.GaussianBlur(gama, [5, 5], 0)
        edges = cv2.Canny(blurred, 50, 150)
        return edges