import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    package_dir = get_package_share_directory('smb_highlevel_controller')
    ld = LaunchDescription()
    param = os.path.join(package_dir, 'config', 'params.yaml')
        
    node=Node(
        package = 'smb_highlevel_controller',
        name = 'p_controller',
        executable = 'p_controller',
        output='screen',
        parameters= [param]
    )
    ld.add_action(node)
    return ld

