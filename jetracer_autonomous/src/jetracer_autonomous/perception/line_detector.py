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
    raw_cross: bool = False
    cross: bool = False
    cross_stable_count: int = 0
    debug: dict = field(default_factory=dict)


class LineDetector:
    def __init__(self, config):
        self.config = config
        self.cross_stable_count = 0

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

        left_score, center_score, right_score, split_boxes = self._score_regions(roi, y_start)
        side_threshold = float(self.config.get("intersection.side_score_threshold", 0.08))
        center_threshold = float(self.config.get("intersection.center_score_threshold", 0.08))
        raw_cross = (
            left_score > side_threshold
            and center_score > center_threshold
            and right_score > side_threshold
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
            raw_cross=raw_cross,
            cross=cross,
            cross_stable_count=self.cross_stable_count,
            debug=debug,
        )

    def _create_mask(self, frame):
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
        else:
            _, mask = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_line_center(self, roi, image_center):
        min_pixels = int(self.config.get("line.min_pixels", 30))
        pixel_count = int(np.count_nonzero(roi))
        if pixel_count < min_pixels:
            return False, None, None

        _, xs = np.nonzero(roi)
        if len(xs) == 0:
            return False, None, None

        line_center = float(np.mean(xs))
        line_error = float(line_center - image_center)
        return True, line_center, line_error

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
