from dataclasses import dataclass, field

try:
    import cv2
except ImportError:  # pragma: no cover - handled on robot or in dependency checks
    cv2 = None

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


@dataclass
class LineInfo:
    found: bool = False
    image_center: float = 0.0
    line_center: float = None
    line_error: float = None
    left_score: float = 0.0
    center_score: float = 0.0
    right_score: float = 0.0
    horizontal_score: float = 0.0
    raw_cross: bool = False
    cross: bool = False
    cross_stable_count: int = 0
    mask_score: float = 0.0
    bottom_score: float = 0.0
    debug: dict = field(default_factory=dict)


class LineDetector:
    def __init__(self, config):
        self.config = config
        self.cross_stable_count = 0
        self.use_cuda = bool(config.get("line.use_cuda", False))
        self.cuda_available = self._check_cuda_available()
        self.cuda_kernel = None
        if self.cuda_available:
            self.cuda_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def process(self, frame):
        if cv2 is None or np is None or frame is None:
            return LineInfo()

        height, width = frame.shape[:2]
        image_center = width / 2.0
        y_start = int(height * float(self.config.get("line.roi_y_start_fraction", 0.55)))
        y_end = int(height * float(self.config.get("line.roi_y_end_fraction", 0.85)))
        y_start = max(0, min(height - 1, y_start))
        y_end = max(y_start + 1, min(height, y_end))

        mask = self._create_mask(frame)
        roi = mask[y_start:y_end, :]
        found, line_center, line_error = self._find_line_center(roi, image_center)
        mask_score = float(np.count_nonzero(roi)) / float(roi.size) if roi.size else 0.0
        bottom_score = self._score_bottom_band(roi)
        if found and not self._bottom_contact_ok(bottom_score):
            found, line_center, line_error = False, None, None

        (
            left_score,
            center_score,
            right_score,
            horizontal_score,
            split_boxes,
        ) = self._score_intersection_regions(
            mask,
            height,
            width,
        )
        side_threshold = float(self.config.get("intersection.side_score_threshold", 0.08))
        center_threshold = float(self.config.get("intersection.center_score_threshold", 0.08))
        horizontal_threshold = float(
            self.config.get("intersection.horizontal_score_threshold", 0.30)
        )
        max_cross_error = float(self.config.get("intersection.max_line_error_for_cross", 99999))
        line_centered = found and line_error is not None and abs(line_error) <= max_cross_error
        split_cross = (
            left_score > side_threshold
            and center_score > center_threshold
            and right_score > side_threshold
        )
        horizontal_cross = (
            horizontal_score > horizontal_threshold
            and center_score > center_threshold
        )
        raw_cross = (
            line_centered
            and (split_cross or horizontal_cross)
        )

        if raw_cross:
            self.cross_stable_count += 1
        else:
            self.cross_stable_count = 0

        stable_frames = int(self.config.get("intersection.cross_stable_frames", 3))
        cross = self.cross_stable_count >= stable_frames

        debug = {
            "mask": mask,
            "roi_box": (0, y_start, width, y_end),
            "split_boxes": split_boxes,
        }

        return LineInfo(
            found=found,
            image_center=image_center,
            line_center=line_center,
            line_error=line_error,
            left_score=left_score,
            center_score=center_score,
            right_score=right_score,
            horizontal_score=horizontal_score,
            raw_cross=raw_cross,
            cross=cross,
            cross_stable_count=self.cross_stable_count,
            mask_score=mask_score,
            bottom_score=bottom_score,
            debug=debug,
        )

    def _create_mask(self, frame):
        if self.cuda_available:
            try:
                return self._create_mask_cuda(frame)
            except Exception:
                self.cuda_available = False
        return self._create_mask_cpu(frame)

    def _create_mask_cpu(self, frame):
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        threshold_value = int(self.config.get("line.threshold_value", 160))
        mode = self.config.get("line.threshold_mode", "white_on_dark")

        if mode == "black_on_light":
            _, mask = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
        elif mode == "auto":
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        elif mode == "adaptive_white_on_dark":
            mask = cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                -5,
            )
        elif mode == "adaptive_black_on_light":
            mask = cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                31,
                5,
            )
        else:
            _, mask = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _create_mask_cuda(self, frame):
        mode = self.config.get("line.threshold_mode", "white_on_dark")
        if mode in ("auto", "adaptive_white_on_dark", "adaptive_black_on_light"):
            return self._create_mask_cpu(frame)

        threshold_value = int(self.config.get("line.threshold_value", 160))
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)

        if len(frame.shape) == 3:
            gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
        else:
            gpu_gray = gpu_frame

        threshold_type = cv2.THRESH_BINARY_INV if mode == "black_on_light" else cv2.THRESH_BINARY
        _, gpu_mask = cv2.cuda.threshold(gpu_gray, threshold_value, 255, threshold_type)

        morph_open = cv2.cuda.createMorphologyFilter(
            cv2.MORPH_OPEN,
            cv2.CV_8UC1,
            self.cuda_kernel,
        )
        morph_close = cv2.cuda.createMorphologyFilter(
            cv2.MORPH_CLOSE,
            cv2.CV_8UC1,
            self.cuda_kernel,
        )
        gpu_mask = morph_open.apply(gpu_mask)
        gpu_mask = morph_close.apply(gpu_mask)
        return gpu_mask.download()

    def _check_cuda_available(self):
        if not self.use_cuda or cv2 is None:
            return False
        try:
            return hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0
        except Exception:
            return False

    def _find_line_center(self, roi, image_center):
        min_pixels = int(self.config.get("line.min_pixels", 30))
        pixel_count = int(np.count_nonzero(roi))
        if pixel_count < min_pixels:
            return False, None, None

        contour_result = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[0] if len(contour_result) == 2 else contour_result[1]
        if not contours:
            return False, None, None

        contour = max(contours, key=cv2.contourArea)
        min_area = float(self.config.get("line.min_contour_area", 80))
        if cv2.contourArea(contour) < min_area:
            return False, None, None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return False, None, None

        line_center = float(moments["m10"] / moments["m00"])
        line_error = float(line_center - image_center)
        return True, line_center, line_error

    def _score_bottom_band(self, roi):
        if roi.size == 0:
            return 0.0

        height = roi.shape[0]
        fraction = float(self.config.get("line.bottom_band_fraction", 0.20))
        band_height = max(1, int(height * fraction))
        bottom = roi[height - band_height : height, :]
        if bottom.size == 0:
            return 0.0
        return float(np.count_nonzero(bottom)) / float(bottom.size)

    def _bottom_contact_ok(self, bottom_score):
        if not bool(self.config.get("line.require_bottom_contact", True)):
            return True
        threshold = float(self.config.get("line.bottom_score_threshold", 0.015))
        return bottom_score >= threshold

    def _score_regions(self, roi, y_start):
        height, width = roi.shape[:2]
        third = max(1, width // 3)
        regions = [
            roi[:, 0:third],
            roi[:, third : 2 * third],
            roi[:, 2 * third : width],
        ]
        scores = []
        for region in regions:
            if region.size == 0:
                scores.append(0.0)
            else:
                scores.append(float(np.count_nonzero(region)) / float(region.size))

        split_boxes = [
            (0, y_start, third, y_start + height),
            (third, y_start, 2 * third, y_start + height),
            (2 * third, y_start, width, y_start + height),
        ]
        return scores[0], scores[1], scores[2], split_boxes

    def _score_intersection_regions(self, mask, image_height, image_width):
        y_start = int(
            image_height * float(self.config.get("intersection.band_y_start_fraction", 0.60))
        )
        y_end = int(
            image_height * float(self.config.get("intersection.band_y_end_fraction", 0.82))
        )
        y_start = max(0, min(image_height - 1, y_start))
        y_end = max(y_start + 1, min(image_height, y_end))

        x_margin = int(
            image_width * float(self.config.get("intersection.side_margin_fraction", 0.08))
        )
        x_margin = max(0, min(image_width // 4, x_margin))

        x_left_start = x_margin
        x_left_end = image_width // 3
        x_center_start = image_width // 3
        x_center_end = 2 * image_width // 3
        x_right_start = 2 * image_width // 3
        x_right_end = image_width - x_margin

        boxes = [
            (x_left_start, y_start, x_left_end, y_end),
            (x_center_start, y_start, x_center_end, y_end),
            (x_right_start, y_start, x_right_end, y_end),
        ]

        band = mask[y_start:y_end, x_margin : image_width - x_margin]
        if band.size:
            band_row_scores = np.count_nonzero(band, axis=1) / float(band.shape[1])
            horizontal_score = float(np.max(band_row_scores))
        else:
            horizontal_score = 0.0

        scores = []
        use_peak_row_score = bool(self.config.get("intersection.use_peak_row_score", True))
        for x1, y1, x2, y2 in boxes:
            region = mask[y1:y2, x1:x2]
            if region.size == 0:
                scores.append(0.0)
            elif use_peak_row_score:
                row_scores = np.count_nonzero(region, axis=1) / float(region.shape[1])
                scores.append(float(np.max(row_scores)))
            else:
                scores.append(float(np.count_nonzero(region)) / float(region.size))

        return scores[0], scores[1], scores[2], horizontal_score, boxes
