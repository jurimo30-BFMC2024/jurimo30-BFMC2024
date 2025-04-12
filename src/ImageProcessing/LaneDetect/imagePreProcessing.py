import cv2
import numpy as np
from skimage.morphology import skeletonize
from numba import njit  # Dodato za ubrzanje

class ImagePreProcessing:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
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

        self.gamma_lut = self._create_gamma_lut(13)  # Precompute LUT for gamma correction
        self.mask = self._create_roi_mask()  # Precompute ROI mask

    def _create_gamma_lut(self, gamma: float):
        """Create a Look-Up Table for gamma correction."""
        lut = np.array([((i / 255.0) ** gamma) * 255 for i in range(256)], dtype=np.uint8)
        return lut

    def _create_roi_mask(self):
        """Precompute the region of interest mask."""
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        cv2.fillPoly(mask, self.roadReg, 255)
        return mask

    def gamma_correction(self, image: np.ndarray, gamma: float = None) -> np.ndarray:
        # Use precomputed LUT for gamma correction
        return cv2.LUT(image, self.gamma_lut)

    @staticmethod
    @njit
    def normalize_histogram(image, lower_percentile=1, upper_percentile=99):
        """Accelerated histogram normalization using Numba."""
        percentiles = np.percentile(image, [lower_percentile, upper_percentile])
        I_min, I_max = percentiles[0], percentiles[1]

        if I_min == I_max:
            return image

        image_normalized = np.clip((image - I_min) / (I_max - I_min), 0, 1) * 255
        return image_normalized.astype(np.uint8)

    def region_of_interest(self, img):
        # Use precomputed mask
        return cv2.bitwise_and(img, self.mask)

    def process_frame(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        temp = self.gamma_correction(gray)  # Use LUT-based gamma correction
        temp = cv2.medianBlur(temp, 5)
        temp = self.region_of_interest(temp)  # Use precomputed ROI mask
        temp = self.normalize_histogram(temp)
        _, bin = cv2.threshold(temp, 230, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Directly skeletonize binary image
        edges = skeletonize(bin // 255)
        edges = (edges * 255).astype(np.uint8)

        return edges