"""
obstacle_detector.py

Detecção de obstáculos (Seção 3.1 "Obstacle Detector" do artigo Bit-Bots).
Depende de color_common.py e dos resultados de field_boundary_detector.py.
"""

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np

from .color_common import ColorDetector


@dataclass
class Obstacle:
    x: int
    y: int
    width: int
    height: int
    color_class: str  # "red", "blue", "white" (goalpost) ou "unknown"

    @property
    def bbox(self) -> Tuple[int, int, int, int]:
        return self.x, self.y, self.width, self.height


def detect_obstacles(
    image_bgr: np.ndarray,
    color_detector: ColorDetector,
    field_boundary_points: List[Tuple[int, int]],
    hull_mask: np.ndarray,
    min_area: int = 150,
) -> List[Obstacle]:
    """
    A região entre o convex hull (campo "ideal") e a fronteira real
    detectada (com dentes causados por obstáculos) é candidata a obstáculo,
    se sua área superar `min_area`. Classificação por cor média: goalpost é
    predominantemente branco, robôs têm a cor do marcador do time.

    Requer os `field_boundary_points` e `hull_mask` já calculados por
    field_boundary_detector.detect_field_boundary().
    """
    h, w = image_bgr.shape[:2]

    real_boundary_mask = np.zeros((h, w), dtype=np.uint8)
    if field_boundary_points:
        polygon = field_boundary_points + [(w - 1, h - 1), (0, h - 1)]
        pts = np.array(polygon, dtype=np.int32)
        cv2.fillPoly(real_boundary_mask, [pts], 255)

    diff_mask = cv2.subtract(hull_mask, real_boundary_mask)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(diff_mask, connectivity=8)

    obstacles: List[Obstacle] = []
    for label_id in range(1, num_labels):  # 0 = fundo
        area = stats[label_id, cv2.CC_STAT_AREA]
        if area < min_area:
            continue

        x = stats[label_id, cv2.CC_STAT_LEFT]
        y = stats[label_id, cv2.CC_STAT_TOP]
        bw = stats[label_id, cv2.CC_STAT_WIDTH]
        bh = stats[label_id, cv2.CC_STAT_HEIGHT]

        roi = image_bgr[y:y + bh, x:x + bw]
        color_class = _classify_by_mean_color(roi, color_detector)

        obstacles.append(Obstacle(x=x, y=y, width=bw, height=bh, color_class=color_class))

    return obstacles


def _classify_by_mean_color(roi_bgr: np.ndarray, color_detector: ColorDetector) -> str:
    """Classificação por contagem de pixels de cada máscara de cor dentro do ROI."""
    if roi_bgr.size == 0:
        return "unknown"

    counts = {
        "red": int(np.count_nonzero(color_detector.red_color.mask(roi_bgr))),
        "blue": int(np.count_nonzero(color_detector.blue_color.mask(roi_bgr))),
        "white": int(np.count_nonzero(color_detector.white_color.mask(roi_bgr))),
    }
    best_class = max(counts, key=counts.get)
    if counts[best_class] == 0:
        return "unknown"
    return best_class
