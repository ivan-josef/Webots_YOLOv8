"""
line_detector.py

Detecção de pontos de marcação de linha (Seção 3.1 "Line Detector" do
artigo Bit-Bots). Depende de color_common.py e dos pontos de fronteira de
field_boundary_detector.py.
"""

from typing import List, Optional, Tuple

import numpy as np

from .color_common import ColorDetector


def detect_line_points(
    image_bgr: np.ndarray,
    color_detector: ColorDetector,
    field_boundary_points: List[Tuple[int, int]],
    n_samples: int = 800,
    rng: Optional[np.random.Generator] = None,
    previous_detections: Optional[List[Tuple[int, int]]] = None,
    density_boost_radius: int = 25,
) -> List[Tuple[int, int]]:
    """
    Segue a abordagem do artigo: retorna PONTOS pertencentes a marcações,
    não linhas geométricas — mais barato computacionalmente e suficiente
    para localização.

    - Amostra pixels aleatoriamente, só abaixo do topo da fronteira do campo.
    - Densidade de amostragem maior em linhas mais baixas na imagem (mais
      perto do robô) e perto de onde houve detecção no frame anterior.
    - Cada amostra é validada contra o espaço de cor das marcações.
    """
    h, w = image_bgr.shape[:2]
    rng = rng or np.random.default_rng()

    if field_boundary_points:
        top_y = min(y for _, y in field_boundary_points)
    else:
        top_y = 0
    top_y = max(0, min(top_y, h - 1))

    marking_mask = color_detector.marking_mask(image_bgr)

    rows = np.arange(top_y, h)
    if rows.size == 0:
        return []
    weights = (rows - top_y + 1).astype(np.float64)
    weights /= weights.sum()

    sampled_y = rng.choice(rows, size=n_samples, p=weights)
    sampled_x = rng.integers(low=0, high=w, size=n_samples)

    points: List[Tuple[int, int]] = []
    for x, y in zip(sampled_x, sampled_y):
        if marking_mask[y, x] > 0:
            points.append((int(x), int(y)))

    if previous_detections:
        extra_samples = max(0, n_samples // 4)
        for _ in range(extra_samples):
            px, py = previous_detections[rng.integers(0, len(previous_detections))]
            x = int(np.clip(px + rng.integers(-density_boost_radius, density_boost_radius + 1), 0, w - 1))
            y = int(np.clip(py + rng.integers(-density_boost_radius, density_boost_radius + 1), top_y, h - 1))
            if marking_mask[y, x] > 0:
                points.append((x, y))

    return points
