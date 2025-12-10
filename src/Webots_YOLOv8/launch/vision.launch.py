# vision.launch.py 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    	
    sim_node = Node(
        package='Webots_YOLOv8',
        executable='finder',
        name='vision',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/camera/image', '/AUREA/CAM/image_color')
        ]
    )


    return LaunchDescription([
        sim_node
    ])
