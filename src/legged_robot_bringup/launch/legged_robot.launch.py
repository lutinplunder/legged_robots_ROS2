import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    legged_robot_parameters = os.path.join(
        get_package_share_directory('legged_robot_bringup'),
        'config',
        'legged_robot_params.yaml'
    )
    legged_robot_param_node = Node(
        namespace='/spider',
        package='legged_robot_bringup',
        executable='legged_robot_parameter_server',
        name='legged_robot_parameter_server',
        parameters=[legged_robot_parameters]
    )
    ld.add_action(legged_robot_param_node)
    return ld
