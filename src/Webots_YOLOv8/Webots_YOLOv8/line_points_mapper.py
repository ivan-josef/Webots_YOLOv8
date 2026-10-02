#!/usr/bin/env python3
"""
Projeta os pontos de linha do pipeline (pixels) para o plano do chao (metros).

O soccer_ipm so mapeia segmentos de marcacao; o artigo Bit-Bots trabalha com
PONTOS de linha. Aqui os pontos sao projetados com o mesmo ipm_library usado
pelo soccer_ipm (mesmo TF buffer / camera_info / plano horizontal) e
publicados como sensor_msgs/PointCloud2 no frame de saida (base_link).
"""

import numpy as np
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

from ipm_library.exceptions import CameraInfoNotSetException, InvalidPlaneException
from ipm_library.ipm import IPM
from soccer_ipm.utils import create_horizontal_plane

import tf2_ros

try:
    from bitbots_tf_buffer import Buffer, TransformListener
except ImportError:
    from tf2_ros import Buffer, TransformListener


class LinePointsMapper:
    def __init__(
        self,
        node,
        output_frame: str = 'base_link',
        camera_info_topic: str = '/AUREA/camera_optical_frame/camera_info',
        topic: str = 'line_points_relative',
        use_distortion: bool = False,
        max_range: float = 10.0,
    ):
        self._node = node
        self._output_frame = output_frame
        self._max_range = max_range
        self._plane = create_horizontal_plane()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._ipm = IPM(self._tf_buffer, distortion=use_distortion)

        node.create_subscription(CameraInfo, camera_info_topic, self._ipm.set_camera_info, 1)
        self._publisher = node.create_publisher(PointCloud2, topic, 1)

    def set_output_frame(self, output_frame: str) -> None:
        self._output_frame = output_frame

    def set_max_range(self, max_range: float) -> None:
        self._max_range = max_range

    def publish(self, points_px, stamp) -> int:
        """Mapeia e publica. Retorna quantos pontos foram publicados."""
        pts = np.asarray(points_px, dtype=np.float32).reshape(-1, 2)
        if pts.shape[0] == 0:
            return 0

        try:
            header, mapped = self._ipm.map_points(
                self._plane,
                pts,
                stamp,
                plane_frame_id=self._output_frame,
                output_frame_id=self._output_frame,
            )
        except CameraInfoNotSetException:
            self._node.get_logger().warn(
                'Line points: ainda sem camera_info, ignorando.', throttle_duration_sec=5)
            return 0
        except InvalidPlaneException:
            return 0
        except (tf2_ros.ConnectivityException,
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warn(
                f'Line points: erro de TF: {e}', throttle_duration_sec=5)
            return 0

        mapped = np.asarray(mapped, dtype=np.float32).reshape(-1, 3)
        # Pontos acima do horizonte nao intersectam o chao (NaN); perto do
        # horizonte a projecao explode, entao tambem se limita o alcance.
        valid = ~np.isnan(mapped).any(axis=1)
        valid &= np.hypot(mapped[:, 0], mapped[:, 1]) <= self._max_range
        mapped = mapped[valid]
        if mapped.shape[0] == 0:
            return 0

        self._publisher.publish(create_cloud_xyz32(header, mapped))
        return int(mapped.shape[0])
