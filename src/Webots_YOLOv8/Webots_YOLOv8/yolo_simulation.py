#!/usr/bin/env python3
import os
import time
from dataclasses import replace

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image as ROS_Image
from vision_msgs.msg import Point2D

import soccer_vision_2d_msgs.msg as sv2dm

import Webots_YOLOv8.running_inference as ri
from Webots_YOLOv8.pipeline import (
    DEFAULT_COLOR_SPACES,
    ColorSpaceHSV,
    PipelineConfig,
    VisionPipeline,
    build_color_detector,
    color_spaces_from_calibration,
    draw_pipeline_overlay,
    filter_points_in_boxes,
    fuse_obstacles_with_detections,
)

# {0: 'ball', 1: 'goalpost', 2: 'robot', 3: 'L-Intersection', 4: 'T-Intersection',
#  5: 'X-Intersection', 6: 'crossbar'}
# Classes do YOLO que "explicam" um obstaculo/ponto de linha do pipeline.
YOLO_KNOWN_OBJECT_CLASSES = (0, 1, 2, 6)

# Classes de cor configuraveis em runtime (marking usa o mesmo espaco do white).
HSV_CLASSES = ('field', 'white', 'red', 'blue')
DEFAULT_WHITE_LOWER = [0, 0, 207]
DEFAULT_WHITE_UPPER = [179, 37, 255]


class YoloSimulacao(Node):

    def __init__(self):
        super().__init__('teste_yolo_sim')
        package_share = get_package_share_directory('Webots_YOLOv8')
        self.get_logger().info('>> MODO TESTE VISUAL (YOLO + PIPELINE) <<')
        self.window_name = "detection window"
        self.source_lut = os.path.join(
            package_share,
            'recursos',
            'green_pixels.csv'
        )
        self.default_model_path = os.path.join(package_share, 'modelo', 'best.pt')

        # 1. Ponte CV <-> ROS
        self.bridge = CvBridge()

        # 2. Parametros + pipeline (fronteira, obstaculos, linhas)
        self._declare_parameters()
        self.pipeline = VisionPipeline(self._build_color_detector_from_params())
        self._hsv_dirty = False
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # 3. YOLO e segmentacao antiga sao opcionais. O pipeline novo funciona
        # mesmo quando ultralytics/modelo ou o segmentador legado nao estao disponiveis.
        self.model = None
        if self.get_parameter('enable_yolo').value:
            model_path = self.get_parameter('model_path').value
            try:
                self.model = ri.load_model(model_path)
                self.get_logger().info(f'Modelo YOLO carregado: {model_path}')
                self.get_logger().info(f'CLASSES DO MODELO: {self.model.names}')
            except Exception as exc:
                self.get_logger().error(
                    f'YOLO desativado porque o modelo nao pode ser carregado: {exc}')

        self.segmentador = None
        self._legacy_load_failed = False
        self._ensure_legacy_segmenter()

        # 4. Subscriber (Entrada do Webots)
        self.camera_subscriber = self.create_subscription(
            ROS_Image,
            '/AUREA/camera_optical_frame/image_color',
            self.image_callback,
            10
        )

        # 5. Publisher DE DEBUG
        self.debug_pub = self.create_publisher(ROS_Image, 'processed_image_topic', 10)

        # 6. Publishers consumidos pelo soccer_ipm (pixels -> metros)

        self.pub_ball = self.create_publisher(sv2dm.BallArray, 'balls_in_image', 1)
        self.pub_goal = self.create_publisher(sv2dm.GoalpostArray, 'goal_posts_in_image', 1)
        self.pub_robot = self.create_publisher(sv2dm.RobotArray, 'robots_in_image', 1)
        self.pub_inter = self.create_publisher(sv2dm.MarkingArray, 'markings_in_image', 1)

        # Saidas do pipeline (o soccer_ipm ja assina estes dois topicos)
        self.pub_boundary = self.create_publisher(
            sv2dm.FieldBoundary, 'field_boundary_in_image', 1)
        self.pub_obstacles = self.create_publisher(
            sv2dm.ObstacleArray, 'obstacles_in_image', 1)

        # Pontos de linha em metros (PointCloud2 em output_frame)
        self.line_mapper = None
        if self.get_parameter('pipeline.publish_line_points_relative').value:
            try:
                from Webots_YOLOv8.line_points_mapper import LinePointsMapper
                self.line_mapper = LinePointsMapper(
                    self,
                    output_frame=self.get_parameter('pipeline.output_frame').value,
                    camera_info_topic=self.get_parameter('pipeline.camera_info_topic').value,
                    max_range=self.get_parameter('pipeline.line_max_range').value,
                )
            except ImportError as e:
                self.get_logger().warn(
                    f'line_points_relative desativado (dependencia ausente: {e}).')

    def _ensure_legacy_segmenter(self):
        """Carrega o segmentador anterior apenas quando ele foi solicitado."""
        if (
            self.segmentador is not None
            or self._legacy_load_failed
            or not self.get_parameter('use_legacy_segmentation').value
        ):
            return
        try:
            from Webots_YOLOv8.segmentacao import Pixel_Segment
            self.segmentador = Pixel_Segment(self.source_lut)
            self.get_logger().info('Segmentacao legada habilitada para o debug.')
        except Exception as exc:
            self._legacy_load_failed = True
            self.get_logger().error(
                f'Segmentacao legada desativada porque nao pode ser carregada: {exc}')

    # ------------------------------------------------------------------
    # Parametros
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        # Derivados da calibracao da simulacao. Se o CSV estiver ausente, o no
        # ainda inicia com faixas HSV conservadoras que podem ser ajustadas.
        if os.path.isfile(self.source_lut):
            spaces = color_spaces_from_calibration(
                self.source_lut,
                white_lower=DEFAULT_WHITE_LOWER,
                white_upper=DEFAULT_WHITE_UPPER,
            )
        else:
            self.get_logger().warn(
                f'CSV de calibracao nao encontrado: {self.source_lut}; usando HSV padrao.')
            spaces = dict(DEFAULT_COLOR_SPACES)
        for cls in HSV_CLASSES:
            lo, hi = spaces[cls].as_min_max()
            self.declare_parameter(f'hsv.{cls}_min', lo)
            self.declare_parameter(f'hsv.{cls}_max', hi)

        cfg = PipelineConfig()
        self.declare_parameter('use_pipeline', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('model_path', self.default_model_path)
        self.declare_parameter('use_legacy_segmentation', False)
        self.declare_parameter('pipeline.process_width', cfg.process_width)
        self.declare_parameter('pipeline.column_step', cfg.column_step)
        self.declare_parameter('pipeline.min_obstacle_area', cfg.min_obstacle_area)
        self.declare_parameter('pipeline.line_samples', 2000)
        self.declare_parameter('pipeline.head_tilted_up', cfg.head_tilted_up)
        self.declare_parameter('pipeline.fuse_with_yolo', True)
        self.declare_parameter('pipeline.publish_line_points_relative', True)
        self.declare_parameter('pipeline.output_frame', 'base_link')
        self.declare_parameter(
            'pipeline.camera_info_topic', '/AUREA/camera_optical_frame/camera_info')
        self.declare_parameter('pipeline.line_max_range', 10.0)

    def _on_set_parameters(self, params):
        for p in params:
            if p.name.startswith('hsv.'):
                v = p.value
                ok = (
                    p.type_ == Parameter.Type.INTEGER_ARRAY
                    and len(v) == 3
                    and 0 <= v[0] <= 179
                    and 0 <= v[1] <= 255
                    and 0 <= v[2] <= 255
                )
                if not ok:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} deve ser [H(0-179), S(0-255), V(0-255)]')
                self._hsv_dirty = True
        return SetParametersResult(successful=True)

    def _build_color_detector_from_params(self):
        spaces = {}
        for cls in HSV_CLASSES:
            spaces[cls] = ColorSpaceHSV.from_min_max(
                list(self.get_parameter(f'hsv.{cls}_min').value),
                list(self.get_parameter(f'hsv.{cls}_max').value),
            )
        spaces['marking'] = spaces['white']
        return build_color_detector(spaces)

    def _refresh_pipeline_config(self):
        # Os valores novos so ficam visiveis aqui, depois do callback de set.
        if self._hsv_dirty:
            self.pipeline.set_color_detector(self._build_color_detector_from_params())
            self._hsv_dirty = False
            self.get_logger().info('Faixas HSV do pipeline atualizadas.')

        gp = self.get_parameter
        self.pipeline.config = PipelineConfig(
            process_width=int(gp('pipeline.process_width').value),
            column_step=int(gp('pipeline.column_step').value),
            min_obstacle_area=int(gp('pipeline.min_obstacle_area').value),
            n_line_samples=int(gp('pipeline.line_samples').value),
            head_tilted_up=bool(gp('pipeline.head_tilted_up').value),
        )
        if self.line_mapper is not None:
            self.line_mapper.set_output_frame(gp('pipeline.output_frame').value)
            self.line_mapper.set_max_range(float(gp('pipeline.line_max_range').value))

    # ------------------------------------------------------------------
    # Callback de imagem
    # ------------------------------------------------------------------

    def image_callback(self, ros_image_msg):
        try:
            # Converte entrada
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")

            # Roda o YOLO
            self._ensure_legacy_segmenter()
            resultado = None
            if (
                self.segmentador is not None
                and self.get_parameter('use_legacy_segmentation').value
            ):
                resultado = self.segmentador.processar(frame)
            classes, scores, boxes, inference_frame = ri.detect_model(self.model, frame)

            if resultado is not None:
                # FUNCAO PARA DESENHAR A SEGMENTACAO
                from Webots_YOLOv8.segmentacao import desenhar_segmentacao
                debug_img = desenhar_segmentacao(
                    inference_frame,
                    resultado
                )
            else:
                debug_img = inference_frame

            # ROS Header

            header = ros_image_msg.header

            # Criação das mensagens vazias

            msg_ball = sv2dm.BallArray(header=header)
            msg_goal = sv2dm.GoalpostArray(header=header)
            msg_robot = sv2dm.RobotArray(header=header)
            msg_int = sv2dm.MarkingArray(header=header)

            # Loop das mensagens

            for i in range(len(boxes)):
                box = boxes[i]  # [x,y,w,h]
                cls = classes[i]
                conf = scores[i]

                # {0: 'ball', 1: 'goalpost', 2: 'robot', 3: 'L-Intersection', 4: 'T-Intersection', 5: 'X-Intersection', 6: 'crossbar'}

                # Bola
                if cls == 0:
                    b = sv2dm.Ball()
                    b.center.x = float(box[0])
                    b.center.y = float(box[1])
                    b.confidence.confidence = conf
                    msg_ball.balls.append(b)

                if cls == 1:
                    g = sv2dm.Goalpost()
                    g.bb.center.position.x = float(box[0])
                    g.bb.center.position.y = float(box[1])
                    g.bb.size_x = float(box[2])
                    g.bb.size_y = float(box[3])
                    g.confidence.confidence = conf
                    msg_goal.posts.append(g)

                if cls == 2:
                    r = sv2dm.Robot()
                    r.bb.center.position.x = float(box[0])
                    r.bb.center.position.y = float(box[1])
                    r.bb.size_x = float(box[2])
                    r.bb.size_y = float(box[3])
                    r.confidence.confidence = conf
                    msg_robot.robots.append(r)

                if cls == 3 or cls == 4 or cls == 5:
                    inter = sv2dm.MarkingIntersection()
                    inter.center.x = float(box[0])
                    inter.center.y = float(box[1])
                    inter.confidence.confidence = conf
                    if cls == 3:  # L
                        inter.num_rays = 2
                    elif cls == 4:  # T
                        inter.num_rays = 3
                    elif cls == 5:  # X
                        inter.num_rays = 4

                    inter.heading_rays = []  # vazio de acordo com a informação da mensagem
                    msg_int.intersections.append(inter)

            # --- PIPELINE (fronteira do campo, obstaculos, pontos de linha) ---
            if self.get_parameter('use_pipeline').value:
                debug_img = self._run_pipeline(frame, header, classes, boxes, debug_img)

            # publicação

            self.pub_ball.publish(msg_ball)
            self.pub_goal.publish(msg_goal)
            self.pub_robot.publish(msg_robot)
            self.pub_inter.publish(msg_int)

            # --- VISUALIZAÇÃO ---
            # Converte a imagem desenhada de volta para ROS e publica
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")  # PUBLICAR DEBUG_IMG (SEGMENTACAO + INFERENCIA + PIPELINE)
            debug_msg.header = ros_image_msg.header

            self.debug_pub.publish(debug_msg)

            if self.get_parameter('show_window').value:
                cv2.imshow(self.window_name, debug_img)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Erro: {e}')

    def _run_pipeline(self, frame, header, classes, boxes, debug_img):
        self._refresh_pipeline_config()

        t0 = time.perf_counter()
        result = self.pipeline.run(frame)

        # Caixas (x0, y0, x1, y1) das deteccoes do YOLO que ja explicam
        # obstaculos e pontos brancos (bola, trave, robo, travessao).
        known_boxes = [
            (bx - bw / 2.0, by - bh / 2.0, bx + bw / 2.0, by + bh / 2.0)
            for (bx, by, bw, bh), c in zip(boxes, classes)
            if int(c) in YOLO_KNOWN_OBJECT_CLASSES
        ]

        obstacles = result.obstacles
        if self.get_parameter('pipeline.fuse_with_yolo').value:
            obstacles = fuse_obstacles_with_detections(obstacles, known_boxes)

        # Pontos de linha nao podem cair sobre objetos (trave/bola/robo/obstaculo).
        obstacle_boxes = [(o.x, o.y, o.x + o.width, o.y + o.height) for o in obstacles]
        line_points = filter_points_in_boxes(
            result.line_points, known_boxes + obstacle_boxes)
        result = replace(result, obstacles=obstacles, line_points=line_points)

        # Fronteira do campo. NAO publicar vazia: o soccer_ipm falha com 0 pontos.
        if result.field_boundary:
            msg_boundary = sv2dm.FieldBoundary(header=header)
            msg_boundary.points = [
                Point2D(x=float(x), y=float(y)) for x, y in result.field_boundary
            ]
            self.pub_boundary.publish(msg_boundary)

        # Obstaculos (lista vazia e valida)
        msg_obstacles = sv2dm.ObstacleArray(header=header)
        for o in result.obstacles:
            ob = sv2dm.Obstacle()
            ob.bb.center.position.x = float(o.x + o.width / 2.0)
            ob.bb.center.position.y = float(o.y + o.height / 2.0)
            ob.bb.size_x = float(o.width)
            ob.bb.size_y = float(o.height)
            msg_obstacles.obstacles.append(ob)
        self.pub_obstacles.publish(msg_obstacles)

        # Pontos de linha em metros
        if self.line_mapper is not None and result.line_points:
            self.line_mapper.publish(result.line_points, header.stamp)

        self.get_logger().info(
            f'pipeline: {1000.0 * (time.perf_counter() - t0):.1f} ms | '
            f'fronteira={len(result.field_boundary)} obstaculos={len(result.obstacles)} '
            f'linhas={len(result.line_points)}',
            throttle_duration_sec=10.0,
        )
        return draw_pipeline_overlay(debug_img, result)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSimulacao()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
