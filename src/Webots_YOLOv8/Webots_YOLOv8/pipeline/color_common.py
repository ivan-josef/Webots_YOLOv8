"""
color_common.py

Modulo compartilhado pelos tres detectores (field_boundary_detector.py,
obstacle_detector.py, line_detector.py). Define o espaco de cor HSV e o
ColorDetector, seguindo a Secao 3.1 "Color Detector" do artigo Bit-Bots.

Alteracoes em relacao ao repositorio original do pipeline:
  * ColorSpaceHSV aceita hue circular (h_min > h_max), necessario para o
    vermelho, que cruza o 0/179 no OpenCV.
  * ColorSpaceHSV.mask_hsv() permite reaproveitar uma imagem ja convertida
    para HSV.
  * color_spaces_from_calibration() deriva as faixas HSV a partir dos CSVs de
    calibracao do Webots (verde do campo) e dos limiares do branco.
"""

from dataclasses import dataclass
from typing import Dict, Sequence

import cv2
import numpy as np
import pandas as pd


@dataclass
class ColorSpaceHSV:
    """Espaco de cor definido por min/max nos 3 canais HSV (OpenCV: H em 0-179).

    Se h_min > h_max a faixa de hue e circular: [h_min, 179] U [0, h_max].
    """
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int

    def mask_hsv(self, image_hsv: np.ndarray) -> np.ndarray:
        s_lo, s_hi = self.s_min, self.s_max
        v_lo, v_hi = self.v_min, self.v_max
        if self.h_min <= self.h_max:
            return cv2.inRange(
                image_hsv,
                np.array([self.h_min, s_lo, v_lo]),
                np.array([self.h_max, s_hi, v_hi]),
            )
        low = cv2.inRange(
            image_hsv, np.array([0, s_lo, v_lo]), np.array([self.h_max, s_hi, v_hi])
        )
        high = cv2.inRange(
            image_hsv, np.array([self.h_min, s_lo, v_lo]), np.array([179, s_hi, v_hi])
        )
        return cv2.bitwise_or(low, high)

    def mask(self, image_bgr: np.ndarray) -> np.ndarray:
        return self.mask_hsv(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV))

    def as_min_max(self):
        """Retorna ([h,s,v] minimos, [h,s,v] maximos) como listas de int."""
        return (
            [int(self.h_min), int(self.s_min), int(self.v_min)],
            [int(self.h_max), int(self.s_max), int(self.v_max)],
        )

    @classmethod
    def from_min_max(cls, lower: Sequence[int], upper: Sequence[int]) -> "ColorSpaceHSV":
        return cls(
            int(lower[0]), int(upper[0]),
            int(lower[1]), int(upper[1]),
            int(lower[2]), int(upper[2]),
        )


class ColorDetector:
    """
    Classifica pixels contra espacos de cor configuraveis: campo (verde),
    marcacoes (branco), e marcadores de time (vermelho/azul).
    """

    def __init__(
        self,
        field_color: ColorSpaceHSV,
        marking_color: ColorSpaceHSV,
        red_color: ColorSpaceHSV,
        blue_color: ColorSpaceHSV,
        white_color: ColorSpaceHSV,
    ):
        self.field_color = field_color
        self.marking_color = marking_color
        self.red_color = red_color
        self.blue_color = blue_color
        self.white_color = white_color

    def field_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        return self.field_color.mask(image_bgr)

    def marking_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        return self.marking_color.mask(image_bgr)


# ---------------------------------------------------------------------------
# Valores de EXEMPLO (usados em testes sinteticos e como padrao para as cores
# de time, ja que o mundo training_simulation.wbt nao tem robos com marcador).
# ---------------------------------------------------------------------------

DEFAULT_COLOR_SPACES: Dict[str, ColorSpaceHSV] = {
    "field": ColorSpaceHSV(35, 85, 50, 255, 30, 255),
    "marking": ColorSpaceHSV(0, 179, 0, 60, 180, 255),
    "red": ColorSpaceHSV(170, 10, 100, 255, 80, 255),  # hue circular
    "blue": ColorSpaceHSV(100, 130, 100, 255, 80, 255),
    "white": ColorSpaceHSV(0, 179, 0, 60, 180, 255),
}


def load_color_spaces_from_csv(csv_path: str = "meus_valores_hsv.csv") -> Dict[str, ColorSpaceHSV]:
    """
    Formato esperado do CSV (uma linha por classe de cor):
        classe,h_min,h_max,s_min,s_max,v_min,v_max
        field,35,85,50,255,30,255
        marking,0,180,0,60,180,255
        red,0,10,100,255,80,255
        blue,100,130,100,255,80,255
        white,0,180,0,60,180,255
    """
    df = pd.read_csv(csv_path)

    color_spaces: Dict[str, ColorSpaceHSV] = {}
    for _, row in df.iterrows():
        color_spaces[row["classe"]] = ColorSpaceHSV(
            h_min=int(row["h_min"]), h_max=int(row["h_max"]),
            s_min=int(row["s_min"]), s_max=int(row["s_max"]),
            v_min=int(row["v_min"]), v_max=int(row["v_max"]),
        )
    return color_spaces


def color_spaces_from_calibration(
    green_csv_path: str,
    white_lower: Sequence[int],
    white_upper: Sequence[int],
    field_margin: Sequence[int] = (8, 40, 50),
    low_quantile: float = 0.01,
    high_quantile: float = 0.99,
) -> Dict[str, ColorSpaceHSV]:
    """
    Deriva os espacos de cor a partir da calibracao ja existente na simulacao.

    * field: faixa [q01, q99] de cada canal do CSV de pixels verdes, alargada
      por `field_margin` (H, S, V). O CSV tem poucos pixels amostrados (a LUT
      original so reconhece exatamente aquelas tuplas); a faixa com margem
      generaliza para variacoes de iluminacao/sombra.
    * white/marking: limiares do branco (os mesmos de def_white_threshold.py).
    * red/blue: valores padrao (nao ha robos com marcador no mundo atual).
    """
    df = pd.read_csv(green_csv_path)
    lo = df[["H", "S", "V"]].quantile(low_quantile)
    hi = df[["H", "S", "V"]].quantile(high_quantile)

    def _clip(v, top):
        return int(np.clip(round(float(v)), 0, top))

    field = ColorSpaceHSV(
        h_min=_clip(lo["H"] - field_margin[0], 179),
        h_max=_clip(hi["H"] + field_margin[0], 179),
        s_min=_clip(lo["S"] - field_margin[1], 255),
        s_max=_clip(hi["S"] + field_margin[1], 255),
        v_min=_clip(lo["V"] - field_margin[2], 255),
        v_max=_clip(hi["V"] + field_margin[2], 255),
    )
    white = ColorSpaceHSV.from_min_max(white_lower, white_upper)

    return {
        "field": field,
        "marking": white,
        "white": white,
        "red": DEFAULT_COLOR_SPACES["red"],
        "blue": DEFAULT_COLOR_SPACES["blue"],
    }


def build_color_detector(color_spaces: Dict[str, ColorSpaceHSV]) -> ColorDetector:
    """Monta um ColorDetector a partir de um dict de espacos de cor (do CSV ou default)."""
    return ColorDetector(
        field_color=color_spaces["field"],
        marking_color=color_spaces["marking"],
        red_color=color_spaces["red"],
        blue_color=color_spaces["blue"],
        white_color=color_spaces["white"],
    )
