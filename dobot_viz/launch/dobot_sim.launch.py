from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    dobot_viz_path = get_package_share_path('dobot_viz')
    default_model_path = dobot_viz_path / 'urdf/dobot_viz.urdf.xacro'
    default_rviz_config_path = dobot_viz_path / 'rviz/dobot_viz.rviz'
    params_path = dobot_viz_path / 'params/robot_size.yaml'

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     parameters=[{
    #         "source_list": ParameterValue(["joint_states"])
    #     }],
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    robot_control = Node(
        package='dobot_viz',
        executable='robot_control',
        name='robot_control',
        output='screen',

    )

    marker_publisher = Node(
        package='dobot_viz',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen',

    )
    # move_to_point = Node(
    #     package='dobot_viz',
    #     executable='move_to_point',
    #     name='move_to_point',
    #     output='screen',

    # )


    return LaunchDescription([
        # gazebo,

        DeclareLaunchArgument(name='rvizconfig',
                              default_value=str(default_rviz_config_path),
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='gui',
                              default_value='true',
                              choices=['true', 'false'],
                              description='Enable joint_state_publisher_gui'),



        # joint_state_publisher_gui_node,
        # marker_publisher,
        # robot_control,
        # move_to_point,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                        Command(['xacro ', str(default_model_path),
                                 ' params_path:=', str(params_path)]),
                        value_type=str),
                        "use_sim_time": True}
                        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]),
        Node(
            package='JoystickBridge',
            executable='joystick_handler',
            name='joystick_handler',
        ),
        Node(
            package='JoystickBridge',
            executable='manip_publisher',
            name='manip_publisher',
        ),
        Node(
            package='ReverseKinematic',
            executable='ReverseKinematic',
            name='ReverseKinematic',
            output='screen'),
            # spawn_entity,
    ])
