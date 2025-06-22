import cv2
import numpy as np
from numba import njit  # Dodato za ubrzanje

class ImagePreProcessing:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc

        # self.roadReg = np.array([[
        #     (int(self.width * 0.01), self.height * 0.45),
        #     (int(self.width * 0.99), self.height * 0.45),
        #     (int(self.width * 0.99), self.height * 0.85),
        #     (int(self.width * 0.01), self.height * 0.85)
        # ]], np.int32)

        self.roadReg = np.array([[
            (int(self.width * 0.02), self.height - int(self.height * 0.02)),   # donji lijevi ugao
            (int(self.width * 0.22), self.height - int(self.height * 0.02)),  # donji lijevi prije ulegnuća
            (int(self.width * 0.30), self.height - int(self.height * 0.35)), # donji lijevi ulegnuće
            (int(self.width * 0.65), self.height - int(self.height * 0.35)), # donji desni ulegnuće
            (int(self.width * 0.73), self.height - int(self.height * 0.02)),  # donji desni prije ulegnuća
            (int(self.width * 0.98), self.height - int(self.height * 0.02)),  # donji desni ugao
            (int(self.width * 0.98), self.height - int(self.height * 0.2)),  
            (int(self.width * 0.8), self.height // 2 - int(self.height * 0.15)), # gornji desni ugao
            (int(self.width * 0.2), self.height // 2 - int(self.height * 0.15)),  # gornji lijevi ugao
            (int(self.width * 0.02), int(self.height*0.7))  # gornji lijevi ugao
        ]], np.int32)

        self.gamma_lut = self._create_gamma_lut(7)  # Precompute LUT for gamma correction
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

    @staticmethod
    @njit # mozda dodati (parallel=True), onda fali paket pip install --upgrade tbb
    def normalize_histogram(image, lower_percentile=1, upper_percentile=99):
        """Accelerated histogram normalization using Numba."""
        # use only roi part of image (compatible with numba)
        # I HATE NUMBA !!!!!!!!!!!!!
        # morao sam 500 puta da izmenjam kod da bi bilo kompatibilno sa ovim sranjem
        # al pazi kad NISTA nije kompatibilno s njitom druuuuuuuuuuuze
        # kod je giga kompleksniji i necitljiviji ali je zato ~3.5x brze u poredjenju sa numpy
        # treba uraditi na raspi pip uninstall opencv-python -> pip install opencv-contrib-python
        # ^^ ovo se radi zbog thinning_zhangshuen algoritma very good bing chilling
        # First collect non-zero pixels (ROI) manually
        roi_pixels = np.zeros(image.size, dtype=image.dtype)  # Pre-allocate
        count = 0
        
        # Manual collection loop (Numba-compatible)
        # efektivno:
        # roi_pixels = image[image > 0]
        # if len(roi_pixels) == 0:
        #     return image
        
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                if image[y, x] > 0:  # ROI check
                    roi_pixels[count] = image[y, x]
                    count += 1
                    
        if count == 0:
            return image  # Return original if no ROI pixels
    
        roi_pixels = roi_pixels[:count]  # Trim to actual size
        
        # Calculate percentiles
        I_min = np.percentile(roi_pixels, lower_percentile)
        I_max = np.percentile(roi_pixels, upper_percentile)
        
        if I_min == I_max:
            return image

        # Normalize image
        # prethodno image_normalized = np.clip((image - I_min) / (I_max - I_min), 0, 1) * 255
        image_normalized = np.empty_like(image, dtype=np.float64)
        range_val = I_max - I_min
        
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                val = image[y, x]
                if val > 0:  # Only process ROI pixels
                    normalized = (val - I_min) / range_val
                    image_normalized[y, x] = max(0.0, min(255.0, normalized * 255.0))
                else:
                    image_normalized[y, x] = 0.0
        
        return image_normalized.astype(np.uint8)

    def process_frame(self, frame: np.ndarray):
        
        processed = np.empty((self.height, self.width), dtype=np.uint8) # empty is faster than zeros and the array values are changed anyways
        # use functions explicitly without making new variables
        cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY, dst=processed)
        cv2.bitwise_and(processed, self.mask, dst=processed) # roi cutoff first, then image processing
        cv2.medianBlur(processed, 7, dst=processed)
        processed = self.normalize_histogram(processed)
        cv2.LUT(processed, self.gamma_lut, dst=processed)
        _, processed = cv2.threshold(processed, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # quicker than scikit
        edges = cv2.ximgproc.thinning(processed, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        return edges