import cv2
import numpy as np
from skimage.morphology import skeletonize

class ImagePreProcessing:
    def __init__(self, width: int, height: int,logging, debugging=False, pc = False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.l245ogging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc

        self.roadReg = np.array([[
            (int(self.width * 0.01), self.height * 0.45),
            (int(self.width * 0.99), self.height * 0.45),
            (int(self.width * 0.99), self.height * 0.85),
            (int(self.width * 0.01), self.height * 0.85)
        ]], np.int32)
    
    def gamma_correction(self, image: np.ndarray, gamma: float) -> np.ndarray:
        image_normalized = image  / 255.0
        corrected = np.power(image_normalized, gamma)
        corrected = np.uint8(corrected * 255)
        return corrected
    
    def normalize_histogram(self, image, lower_percentile=1, upper_percentile=99):
        I_min = np.percentile(image, lower_percentile)
        I_max = np.percentile(image, upper_percentile)

        if I_min == I_max:
            return image

        # Normalizacija
        image_normalized = np.clip((image - I_min) / (I_max - I_min), 0, 1) * 255
        image_normalized = image_normalized.astype(np.uint8)  # Pretvori u 8-bitni format

        return image_normalized
    
    def region_of_interest(self, img):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, self.roadReg, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    
    def process_frame(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gama = self.gamma_correction(gray, 18)
        blurred = cv2.medianBlur(gama, 5)
        temp = self.region_of_interest(blurred)
        temp = self.normalize_histogram(temp)
        _, bin = cv2.threshold(temp, 230, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        xbin = bin / 255 # skimage radi sa binarnim vrednostima (0 ili 1)

        edges = skeletonize(xbin)
        edges = (edges * 255).astype(np.uint8)

        return edges