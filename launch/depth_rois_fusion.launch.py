from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def load_param_yaml_section(path, section):
    with open(path, 'r') as f:
        all_params = yaml.safe_load(f)
    return all_params.get(section, {})

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('depth_rois_fusion'),
        'config',
        'config.yaml'
    )

    params = load_param_yaml_section(config_path, 'depth_rois_fusion_node')

    return LaunchDescription([
        Node(
            package='depth_rois_fusion',
            executable='depth_rois_fusion_node',
            name='depth_rois_fusion_node',
            output='screen',
            parameters=[params]
        )
    ])

