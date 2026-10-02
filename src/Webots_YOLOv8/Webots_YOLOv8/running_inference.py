#!/usr/bin/env python3
"""Carregamento e inferencia do modelo YOLO sem efeitos colaterais no import."""

import os
import time

try:
    from ultralytics import YOLO
except ImportError as exc:  # Permite executar somente o pipeline de segmentacao.
    YOLO = None
    _ULTRALYTICS_IMPORT_ERROR = exc
else:
    _ULTRALYTICS_IMPORT_ERROR = None

IMAGE_SIZE = 320


def load_model(model_path):
    """Carrega explicitamente um modelo, produzindo erros faceis de diagnosticar."""
    if YOLO is None:
        raise RuntimeError(
            "O pacote Python 'ultralytics' nao esta instalado. "
            "Instale-o ou inicie o no com enable_yolo:=false."
        ) from _ULTRALYTICS_IMPORT_ERROR
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f'Modelo YOLO nao encontrado: {model_path}')
    return YOLO(model_path)


def detect_model(model, current_frame):
    """Executa a inferencia; sem modelo, devolve uma deteccao vazia."""
    if model is None:
        return [], [], [], current_frame

    start_time = time.perf_counter()
    results = model.predict(
        source=current_frame,
        conf=0.45,
        imgsz=IMAGE_SIZE,
        max_det=10,
        verbose=False,
        iou=0.5,
    )

    classes = results[0].boxes.cls.tolist()
    scores = results[0].boxes.conf.tolist()
    boxes = results[0].boxes.xywh.tolist()
    _ = time.perf_counter() - start_time
    inference_frame = results[0].plot()
    return classes, scores, boxes, inference_frame
