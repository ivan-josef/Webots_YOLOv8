"""Testes do pipeline de visao (sem ROS): `pytest src/Webots_YOLOv8/test/test_vision_pipeline.py`."""

import os

import cv2
import numpy as np
import pytest

from Webots_YOLOv8.pipeline import (
    ColorSpaceHSV,
    Obstacle,
    PipelineConfig,
    VisionPipeline,
    build_color_detector,
    color_spaces_from_calibration,
    filter_points_in_boxes,
    fuse_obstacles_with_detections,
)

GREEN_CSV = os.path.join(
    os.path.dirname(__file__), '..', 'recursos', 'green_pixels.csv')
WHITE_LOWER, WHITE_UPPER = [0, 0, 207], [179, 37, 255]
W, H, HORIZON = 1080, 720, 360


def _bgr(h, s, v):
    return cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()


def make_scene(with_field=True, seed=0):
    """Camera horizontal: ceu em cima, campo verde embaixo, linhas, trave e robo."""
    rng = np.random.default_rng(seed)
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = _bgr(105, 120, 220)
    if not with_field:
        return img
    field = np.zeros((H - HORIZON, W, 3), np.int16)
    field[:] = _bgr(57, 160, 140)
    field += rng.integers(-6, 7, field.shape, dtype=np.int16)
    img[HORIZON:] = np.clip(field, 0, 255).astype(np.uint8)
    white = (255, 255, 255)
    cv2.rectangle(img, (0, 520), (W - 1, 526), white, -1)       # linha horizontal
    cv2.rectangle(img, (300, HORIZON), (306, H - 1), white, -1)  # linha vertical
    cv2.rectangle(img, (700, 250), (724, 480), white, -1)        # trave, base em y=480
    cv2.rectangle(img, (850, 300), (920, 470), _bgr(0, 200, 200), -1)  # robo vermelho
    return img


@pytest.fixture(scope='module')
def detector():
    spaces = color_spaces_from_calibration(GREEN_CSV, WHITE_LOWER, WHITE_UPPER)
    return build_color_detector(spaces)


def test_calibration_field_range_contains_samples(detector):
    f = detector.field_color
    assert f.h_min <= 57 <= f.h_max
    assert f.s_min <= 160 <= f.s_max
    assert f.v_min <= 140 <= f.v_max


def test_red_hue_wraps_around():
    red = ColorSpaceHSV(170, 10, 100, 255, 80, 255)
    hsv = np.array([[[2, 200, 200], [175, 200, 200], [90, 200, 200]]], np.uint8)
    assert red.mask_hsv(hsv).tolist() == [[255, 255, 0]]


def test_pipeline_finds_boundary_obstacles_and_lines(detector):
    pipeline = VisionPipeline(detector, PipelineConfig(n_line_samples=3000), seed=0)
    res = pipeline.run(make_scene())

    assert res.field_detected
    ys = [y for _, y in res.field_boundary]
    assert min(ys) == pytest.approx(HORIZON, abs=4)

    # Coordenadas na resolucao ORIGINAL (1080x720), nao na reduzida.
    assert max(x for x, _ in res.field_boundary) > W * 0.9

    classes = {o.color_class: o for o in res.obstacles}
    assert 'white' in classes and 'red' in classes
    post, robot = classes['white'], classes['red']
    assert post.x == pytest.approx(700, abs=12)
    assert post.y + post.height == pytest.approx(480, abs=8)   # base da trave
    assert robot.x == pytest.approx(850, abs=12)
    assert robot.y + robot.height == pytest.approx(470, abs=8)

    # Pontos de linha caem em pixels brancos (com tolerancia da reducao).
    white = detector.marking_mask(make_scene())
    white = cv2.dilate(white, np.ones((9, 9), np.uint8))
    assert len(res.line_points) > 20
    on_white = sum(1 for x, y in res.line_points if white[min(y, H - 1), min(x, W - 1)] > 0)
    assert on_white / len(res.line_points) > 0.9


def test_no_field_returns_empty_result(detector):
    res = VisionPipeline(detector, seed=0).run(make_scene(with_field=False))
    assert not res.field_detected
    assert res.field_boundary == [] and res.obstacles == [] and res.line_points == []


def test_runs_at_native_resolution_too(detector):
    res = VisionPipeline(detector, PipelineConfig(process_width=0), seed=0).run(make_scene())
    assert res.field_detected and res.obstacles


def test_fuse_drops_obstacles_explained_by_yolo():
    post = Obstacle(x=700, y=360, width=30, height=122, color_class='white')
    unknown = Obstacle(x=100, y=380, width=40, height=60, color_class='unknown')
    yolo_goalpost = (698.0, 250.0, 726.0, 482.0)
    kept = fuse_obstacles_with_detections([post, unknown], [yolo_goalpost])
    assert kept == [unknown]
    assert fuse_obstacles_with_detections([post, unknown], []) == [post, unknown]


def test_fuse_uses_footpoint_when_overlap_is_small():
    tall = Obstacle(x=100, y=0, width=20, height=400, color_class='white')
    yolo_box = (90.0, 350.0, 130.0, 410.0)   # cobre so a base
    assert fuse_obstacles_with_detections([tall], [yolo_box]) == []


def test_filter_points_in_boxes():
    pts = [(10, 10), (50, 50), (200, 200)]
    out = filter_points_in_boxes(pts, [(40, 40, 60, 60)], margin_px=0)
    assert out == [(10, 10), (200, 200)]
    assert filter_points_in_boxes(pts, []) == pts
    assert filter_points_in_boxes([], [(0, 0, 1, 1)]) == []
