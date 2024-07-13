import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
def generate_launch_description():

    mixer_config = launch.substitutions.LaunchConfiguration('mixer_config')
    mixer_config_filepath = launch.substitutions.LaunchConfiguration('mixer_config_filepath')

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument('mixer_config', default_value='artoo_diff_drive'),
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
        
        # Adding joy_node
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # Specify the device file for the joystick
                'deadzone': 0.05
            }]
        ),
        
        # Adding teleop_twist_joy Node (XBOX Config)
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear': 0,  # Typically, 1 is the left stick up/down axis for linear motion
                'axis_angular': 1,  # Typically, 0 is the left stick left/right axis for angular motion
                'scale_linear': 0.7,
                'scale_linear_turbo': 1.5,
                'scale_angular': 0.4,
                'enable_button': 7,  # Button to enable teleoperation (e.g., X button on joystick)
                'enable_turbo_button': 6,   # Button to enable turbo mode (e.g., B button on joystick)
                'dev': '/dev/input/js0'  # Specify the device file for the joystick
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel')  # Ensure this matches the rest of your setup
            ]
        ),
    ])

