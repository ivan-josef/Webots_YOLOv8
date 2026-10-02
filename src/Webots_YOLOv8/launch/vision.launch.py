# vision.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('Webots_YOLOv8'),
        'config',
        'vision_pipeline.yaml',
    )

    # Argumentos de linha de comando, ex.:
    #   ros2 launch Webots_YOLOv8 vision.launch.py show_window:=false enable_yolo:=false
    show_window = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='Abre a janela do OpenCV com a imagem de debug')
    use_pipeline = DeclareLaunchArgument(
        'use_pipeline', default_value='true',
        description='Liga o pipeline (fronteira do campo, obstaculos, linhas)')
    enable_yolo = DeclareLaunchArgument(
        'enable_yolo', default_value='true',
        description='Carrega e executa o modelo YOLOv8')
    use_legacy_segmentation = DeclareLaunchArgument(
        'use_legacy_segmentation', default_value='false',
        description='Mostra tambem o algoritmo de segmentacao anterior')

    sim_node = Node(
        package='Webots_YOLOv8',
        executable='finder',
        name='vision_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                'use_pipeline': ParameterValue(LaunchConfiguration('use_pipeline'), value_type=bool),
                'enable_yolo': ParameterValue(LaunchConfiguration('enable_yolo'), value_type=bool),
                'use_legacy_segmentation': ParameterValue(
                    LaunchConfiguration('use_legacy_segmentation'), value_type=bool),
            },
        ],
    )

    soccer_ipm = Node(
        package='soccer_ipm',
        executable='ipm',
        name='soccer_ipm_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_frame': 'base_link'},
            {'camera_frame': 'camera_optical_frame'},
            {'balls.ball_diameter': 0.14},
            {'use_distortion': False},
            {'obstacles.footpoint_out_of_image_threshold': 0.8},
        ],
        remappings=[
            ('camera/image_raw', '/AUREA/camera_optical_frame/image_color'),
            ('camera_info', '/AUREA/camera_optical_frame/camera_info')
        ]
    )

    return LaunchDescription([
        show_window,
        use_pipeline,
        enable_yolo,
        use_legacy_segmentation,
        sim_node,
        soccer_ipm,
    ])
