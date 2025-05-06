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
                (int(self.width * 0.01), self.height - int(self.height * 0.05)),
                (int(self.width * 0.25), self.height - int(self.height * 0.05)),
                (int(self.width * 0.3), self.height - int(self.height * 0.2)),
                (int(self.width * 0.7), self.height - int(self.height * 0.2)),
                (int(self.width * 0.75), self.height - int(self.height * 0.05)),
                (int(self.width * 0.99), self.height - int(self.height * 0.05)),
                (int(self.width * 0.99), self.height * 0.45),
                (int(self.width * 0.01), self.height * 0.45)
            ]], np.int32)

        self.gamma_lut = self._create_gamma_lut(13)  # Precompute LUT for gamma correction
        self.mask = self._create_roi_mask()  # Precompute ROI mask

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

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
        processed = np.empty((self.height, self.width), dtype=np.uint8)
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dst=processed)
        cv2.LUT(processed, self.gamma_lut, dst=processed)

		# (jednostavniji; nije minimalan) bounding rect oko roi-a 
        x, y, w, h = cv2.boundingRect(self.mask)

		# crop both image and mask (mask is cropped to be applied to the cropped image)
        crop_img = processed[y:y+h, x:x+w]
        crop_mask = self.mask[y:y+h, x:x+w]

		# CLAHE on cropped image (without mask applied)
        clahe_applied = self.clahe.apply(crop_img)

		# apply mask AFTER CLAHE to avoid useravanje
        clahe_applied[crop_mask == 0] = 0

		# put everything back into original frame size
        full_processed = np.zeros_like(processed)
        full_processed[y:y+h, x:x+w] = clahe_applied

        cv2.medianBlur(full_processed, 5, dst=full_processed)
		# full_processed = self.normalize_histogram(full_processed) (rip big boss)
        _, full_processed = cv2.threshold(full_processed, 230, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.ximgproc.thinning(full_processed, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        return edges