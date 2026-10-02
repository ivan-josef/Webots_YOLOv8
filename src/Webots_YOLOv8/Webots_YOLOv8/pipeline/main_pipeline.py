"""
main_pipeline.py

Orquestra os tres filtros (fronteira do campo -> obstaculos -> pontos de
linha), equivalente ao "main vision module" do artigo Bit-Bots (Fig. 2).

Diferencas em relacao ao repositorio original do pipeline:
  * Nao ha mais efeitos colaterais no import (detector global / imagem fixa):
    tudo vive em `VisionPipeline`, que o no ROS instancia.
  * A imagem e reduzida para `process_width` antes de rodar os filtros (a
    camera do Webots e 1080x720) e os resultados voltam em coordenadas da
    imagem ORIGINAL, que e o que o IPM espera (camera_info = 1080x720).
  * Funcoes de fusao com o YOLO: obstaculos ja explicados por uma detecao do
    YOLO (trave/robo/bola) sao descartados e pontos de linha que caem sobre
    detecoes ou obstaculos tambem.

Uso offline (ajuste de HSV em imagens salvas):
    python -m Webots_YOLOv8.pipeline.main_pipeline imagem.png [green_pixels.csv]
"""

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .color_common import ColorDetector
from .field_boundary_detector import detect_field_boundary
from .line_detector import detect_line_points
from .obstacle_detector import Obstacle, detect_obstacles

Point = Tuple[int, int]
BoxXYXY = Tuple[float, float, float, float]


@dataclass
class PipelineConfig:
    process_width: int = 540        # largura de processamento (0 = sem reducao)
    column_step: int = 4            # 1 coluna a cada N na varredura da fronteira
    min_obstacle_area: int = 150    # area minima (px^2, na resolucao de processamento)
    n_line_samples: int = 800       # amostras aleatorias para os pontos de linha
    head_tilted_up: bool = False    # True -> varredura de baixo para cima
    use_previous_line_points: bool = True


@dataclass
class PipelineResult:
    """Todos os pontos/caixas estao em coordenadas da imagem ORIGINAL."""
    field_boundary: List[Point] = field(default_factory=list)
    obstacles: List[Obstacle] = field(default_factory=list)
    line_points: List[Point] = field(default_factory=list)
    hull_mask: Optional[np.ndarray] = None  # resolucao de processamento
    field_detected: bool = False


class VisionPipeline:
    def __init__(
        self,
        color_detector: ColorDetector,
        config: Optional[PipelineConfig] = None,
        seed: Optional[int] = None,
    ):
        self.color_detector = color_detector
        self.config = config or PipelineConfig()
        self._rng = np.random.default_rng(seed)
        self._previous_line_points: List[Point] = []  # resolucao de processamento

    def set_color_detector(self, color_detector: ColorDetector) -> None:
        self.color_detector = color_detector
        self._previous_line_points = []

    def run(self, image_bgr: np.ndarray) -> PipelineResult:
        cfg = self.config
        h, w = image_bgr.shape[:2]

        if cfg.process_width > 0 and w > cfg.process_width:
            pw = int(cfg.process_width)
            ph = max(1, int(round(h * pw / w)))
            small = cv2.resize(image_bgr, (pw, ph), interpolation=cv2.INTER_AREA)
        else:
            small = image_bgr
        sh, sw = small.shape[:2]
        sx, sy = w / sw, h / sh

        boundary, hull_mask = detect_field_boundary(
            small,
            self.color_detector,
            column_step=max(1, int(cfg.column_step)),
            search_from_top=not cfg.head_tilted_up,
        )

        if len(boundary) < 3:
            # Sem campo na imagem (ex.: camera virada para a parede): nao ha
            # fronteira, obstaculos nem linhas a reportar. Evita amostrar o
            # branco de paredes/fundo como se fossem linhas do campo.
            self._previous_line_points = []
            return PipelineResult(hull_mask=hull_mask, field_detected=False)

        obstacles = detect_obstacles(
            small,
            self.color_detector,
            boundary,
            hull_mask,
            min_area=int(cfg.min_obstacle_area),
        )
        line_points = detect_line_points(
            small,
            self.color_detector,
            boundary,
            n_samples=int(cfg.n_line_samples),
            rng=self._rng,
            previous_detections=(
                self._previous_line_points if cfg.use_previous_line_points else None
            ),
        )
        self._previous_line_points = line_points

        return PipelineResult(
            field_boundary=[(int(round(x * sx)), int(round(y * sy))) for x, y in boundary],
            obstacles=[
                Obstacle(
                    x=int(round(o.x * sx)),
                    y=int(round(o.y * sy)),
                    width=int(round(o.width * sx)),
                    height=int(round(o.height * sy)),
                    color_class=o.color_class,
                )
                for o in obstacles
            ],
            line_points=[(int(round(x * sx)), int(round(y * sy))) for x, y in line_points],
            hull_mask=hull_mask,
            field_detected=True,
        )


# ---------------------------------------------------------------------------
# Fusao com as deteccoes do YOLO
# ---------------------------------------------------------------------------

def _obstacle_xyxy(o: Obstacle) -> BoxXYXY:
    return (o.x, o.y, o.x + o.width, o.y + o.height)


def fuse_obstacles_with_detections(
    obstacles: Sequence[Obstacle],
    detections_xyxy: Iterable[BoxXYXY],
    coverage_threshold: float = 0.3,
    footpoint_margin: float = 0.1,
) -> List[Obstacle]:
    """
    Mantem so os obstaculos NAO explicados por uma deteccao conhecida.

    Na mensagem soccer_vision_2d_msgs/Obstacle, "obstaculo" e tudo que nao foi
    classificado como robo/trave. Um obstaculo do pipeline e descartado se:
      * pelo menos `coverage_threshold` da sua area esta dentro de uma caixa
        do YOLO, ou
      * o ponto de apoio dele (centro da base) cai dentro de uma caixa do YOLO
        (expandida em `footpoint_margin`).
    """
    dets = list(detections_xyxy)
    kept: List[Obstacle] = []
    for o in obstacles:
        ox0, oy0, ox1, oy1 = _obstacle_xyxy(o)
        area = max(1.0, float((ox1 - ox0) * (oy1 - oy0)))
        foot_x, foot_y = (ox0 + ox1) / 2.0, oy1

        explained = False
        for x0, y0, x1, y1 in dets:
            iw = max(0.0, min(ox1, x1) - max(ox0, x0))
            ih = max(0.0, min(oy1, y1) - max(oy0, y0))
            if iw * ih / area >= coverage_threshold:
                explained = True
                break
            mx, my = footpoint_margin * (x1 - x0), footpoint_margin * (y1 - y0)
            if x0 - mx <= foot_x <= x1 + mx and y0 - my <= foot_y <= y1 + my:
                explained = True
                break
        if not explained:
            kept.append(o)
    return kept


def filter_points_in_boxes(
    points: Sequence[Point],
    boxes_xyxy: Iterable[BoxXYXY],
    margin_px: float = 4.0,
) -> List[Point]:
    """Remove pontos que caem dentro de qualquer caixa (expandida em margin_px)."""
    boxes = list(boxes_xyxy)
    if not points or not boxes:
        return list(points)

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    inside = np.zeros(len(pts), dtype=bool)
    for x0, y0, x1, y1 in boxes:
        inside |= (
            (pts[:, 0] >= x0 - margin_px) & (pts[:, 0] <= x1 + margin_px)
            & (pts[:, 1] >= y0 - margin_px) & (pts[:, 1] <= y1 + margin_px)
        )
    return [p for p, drop in zip(points, inside) if not drop]


# ---------------------------------------------------------------------------
# Debug
# ---------------------------------------------------------------------------

_OBSTACLE_COLORS = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "white": (255, 255, 255),
    "unknown": (0, 255, 255),
}
BOUNDARY_COLOR = (0, 165, 255)   # laranja (BGR)
LINE_POINT_COLOR = (255, 255, 0)  # ciano (BGR)


def draw_pipeline_overlay(image_bgr: np.ndarray, result: PipelineResult) -> np.ndarray:
    """Desenha fronteira, obstaculos e pontos de linha (similar a Fig. 1 do artigo)."""
    debug = image_bgr.copy()

    if len(result.field_boundary) >= 2:
        pts = np.array(result.field_boundary, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(debug, [pts], isClosed=False, color=BOUNDARY_COLOR, thickness=2)

    for x, y in result.line_points:
        cv2.circle(debug, (x, y), 2, LINE_POINT_COLOR, -1)

    for o in result.obstacles:
        color = _OBSTACLE_COLORS.get(o.color_class, _OBSTACLE_COLORS["unknown"])
        cv2.rectangle(debug, (o.x, o.y), (o.x + o.width, o.y + o.height), color, 2)
        cv2.putText(
            debug, f"obs:{o.color_class}", (o.x, max(12, o.y - 4)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
        )

    status = "Pipeline: campo OK" if result.field_detected else "Pipeline: sem campo"
    cv2.putText(
        debug, status, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
        (0, 255, 0) if result.field_detected else (0, 0, 255), 2,
    )
    return debug


def _demo(argv: Sequence[str]) -> int:
    from .color_common import (
        DEFAULT_COLOR_SPACES,
        build_color_detector,
        load_color_spaces_from_csv,
    )

    if len(argv) < 2:
        print(__doc__)
        return 1
    image = cv2.imread(argv[1])
    if image is None:
        print(f"Nao consegui abrir a imagem: {argv[1]}")
        return 1

    spaces = DEFAULT_COLOR_SPACES
    if len(argv) > 2:
        spaces = load_color_spaces_from_csv(argv[2])
    pipeline = VisionPipeline(build_color_detector(spaces), seed=0)
    result = pipeline.run(image)

    print(f"Campo detectado: {result.field_detected}")
    print(f"Pontos de fronteira: {len(result.field_boundary)}")
    print(f"Obstaculos: {len(result.obstacles)}")
    for o in result.obstacles:
        print(f"  - {o.color_class} em {o.bbox}")
    print(f"Pontos de linha: {len(result.line_points)}")
    cv2.imwrite("debug_output.jpg", draw_pipeline_overlay(image, result))
    print("Imagem de debug salva em debug_output.jpg")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(_demo(sys.argv))
