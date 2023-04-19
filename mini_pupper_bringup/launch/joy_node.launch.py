import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('mini_pupper_bringup'),
                              'config'
                              )

    joy_config_filepath = os.path.join(
        config_dir,
        "joy.config.yaml"
    )

    joy_node = Node(package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[joy_config_filepath]
                    )

    return launch.LaunchDescription([joy_node])
