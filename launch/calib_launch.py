from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz (its config is FAST-LIVO2\'s; Foxglove is the viewer here)'
    )

    # Which parameter file to run. DEFAULT IS THE ROVER (Airy + ZED 2i) -- running with
    # the wrong file produces a plausible-looking but wrong extrinsic and no error
    # message, so the config that is actually in use is the one that needs no argument.
    # Go2: params_file:=<pkg_share>/config/qr_params_go2.yaml
    # Mid360 sample-data regression case: params_file:=<pkg_share>/config/qr_params.yaml
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('fast_calib'), 'config', 'qr_params_rover.yaml'),
        description='Absolute path to the qr_params YAML to use '
                    '(default: qr_params_rover.yaml, Airy + ZED 2i, Go2 half-scale board)'
    )

    # Get package directory
    pkg_share = get_package_share_directory('fast_calib')
    
    # Parameters file path (overridable via the params_file launch argument)
    params_file = LaunchConfiguration('params_file')
    
    # RViz config file path
    rviz_config = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')

    # Fast calib node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        parameters=[params_file],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        params_arg,
        fast_calib_node,
        rviz_node
    ])