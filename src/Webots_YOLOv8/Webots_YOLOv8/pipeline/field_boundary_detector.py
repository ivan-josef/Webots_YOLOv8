"""
field_boundary_detector.py

Detecção da fronteira do campo (Seção 3.1 "Field Boundary Detector" do
artigo Bit-Bots). Depende de color_common.py.
"""

from typing import List, Tuple

import cv2
import numpy as np

from .color_common import ColorDetector


def detect_field_boundary(
    image_bgr: np.ndarray,
    color_detector: ColorDetector,
    column_step: int = 4,
    search_from_top: bool = True,
    kernel_size: int = 5,
) -> Tuple[List[Tuple[int, int]], np.ndarray]:
    """
    Varredura por colunas (scanline), não pixel a pixel — otimização de
    runtime citada no artigo (só 1 a cada `column_step` colunas é checada).

    - search_from_top=True: de cima para baixo até achar verde. Rápido,
      usado quando o campo ocupa a maior parte da imagem.
    - search_from_top=False: de baixo para cima até achar não-verde, com
      kernel (morphological close) para não confundir linhas brancas
      internas com a borda externa. Mais lento; usado quando a cabeça do
      robô está inclinada para cima.

    Depois calcula o convex hull dos pontos de fronteira, para eliminar
    "dentes" causados por obstáculos que obstruem a borda real.

    Retorna (pontos_por_coluna, mascara_do_hull).
    """
    h, w = image_bgr.shape[:2]

    field_mask = color_detector.field_mask(image_bgr)

    if not search_from_top:
        k = np.ones((kernel_size, kernel_size), np.uint8)
        field_mask = cv2.morphologyEx(field_mask, cv2.MORPH_CLOSE, k)

    boundary_points: List[Tuple[int, int]] = []

    for x in range(0, w, column_step):
        col = field_mask[:, x]
        if search_from_top:
            green_rows = np.flatnonzero(col)
            if green_rows.size > 0:
                y = int(green_rows[0])
                boundary_points.append((x, y))
        else:
            for y in range(h - 1, -1, -1):
                if col[y] == 0:
                    boundary_points.append((x, y))
                    break
            else:
                boundary_points.append((x, 0))

    hull_mask = np.zeros((h, w), dtype=np.uint8)
    if len(boundary_points) >= 3:
        polygon = boundary_points + [(w - 1, h - 1), (0, h - 1)]
        pts = np.array(polygon, dtype=np.int32)
        hull = cv2.convexHull(pts)
        cv2.fillConvexPoly(hull_mask, hull, 255)

    return boundary_points, hull_mask
