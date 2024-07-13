import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
def generate_launch_description():

    mixer_config = launch.substitutions.LaunchConfiguration('mixer_config')
    mixer_config_filepath = launch.substitutions.LaunchConfiguration('mixer_config_filepath')

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument('mixer_config', default_value='pwm_diff_drive'),
        launch.actions.DeclareLaunchArgument('mixer_config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('pwm_pca9685'), 'config', '')),
            mixer_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='pwm_pca9685', executable='mixer_node',
            name='mixer_node',
            remappings=[
                    ('/cmd_vel', '/cmd_vel'),
                    ('/command', '/command'),
            ],
            parameters=[mixer_config_filepath]
        ),

        launch_ros.actions.Node(
            package='pwm_pca9685', executable='pca9685_node',
            name='pwm_node',
            remappings=[
                    ('/command', '/command'),
            ],
            parameters=[mixer_config_filepath]
        ),
    ])

