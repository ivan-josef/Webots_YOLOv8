"""
Vision pipeline (pipe-and-filter) inspirado no artigo Hamburg Bit-Bots.

Este subpacote NAO depende de ROS: pode ser testado com `pytest` puro.
A cola com o ROS 2 fica em `Webots_YOLOv8.yolo_simulation`.
"""

from .color_common import (
    DEFAULT_COLOR_SPACES,
    ColorDetector,
    ColorSpaceHSV,
    build_color_detector,
    color_spaces_from_calibration,
    load_color_spaces_from_csv,
)
from .field_boundary_detector import detect_field_boundary
from .line_detector import detect_line_points
from .main_pipeline import (
    PipelineConfig,
    PipelineResult,
    VisionPipeline,
    draw_pipeline_overlay,
    filter_points_in_boxes,
    fuse_obstacles_with_detections,
)
from .obstacle_detector import Obstacle, detect_obstacles

__all__ = [
    "ColorDetector", "ColorSpaceHSV", "DEFAULT_COLOR_SPACES", "Obstacle",
    "PipelineConfig", "PipelineResult", "VisionPipeline",
    "build_color_detector", "color_spaces_from_calibration",
    "detect_field_boundary", "detect_line_points", "detect_obstacles",
    "draw_pipeline_overlay", "filter_points_in_boxes",
    "fuse_obstacles_with_detections", "load_color_spaces_from_csv",
]
