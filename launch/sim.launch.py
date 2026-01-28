from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_bot')

    world = os.path.join(pkg_share, 'worlds', 'rtab_map_stage_world2.sdf')
    # world = os.path.join(pkg_share, 'worlds', 'mecanum_worlds.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_bot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_file_arg = DeclareLaunchArgument(
        'robot_file',
        default_value=xacro_file,
        description='URDF/Xacro file to load for robot_description'
    )

    robot_file = LaunchConfiguration('robot_file')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', robot_file,
                ' controllers_yaml:=', controllers_yaml
            ]),
            value_type=str,
        )
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'mecanum_bot', '-param', 'robot_description'],
        parameters=[robot_description],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'mecanum_bot/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # lidar_laser_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser_frame'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen',
    # )

    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    cmd_vel_scaler = Node(
        package='mecanum_bot',
        executable='cmd_vel_scaler.py',
        name='cmd_vel_scaler',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/mecanum_controller/reference_unstamped',
            'scale_linear': 0.001,
            'scale_angular': 0.001,
            'max_input': 500.0,
            'use_sim_time': True,
        }],
    )

    slip_monitor = Node(
        package='mecanum_bot',
        executable='slip_monitor.py',
        name='slip_monitor',
        output='screen',
        parameters=[{
            'wheel_radius': 0.05,
            'sum_lw': 0.35,
            'joint_names': [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'rear_left_wheel_joint',
                'rear_right_wheel_joint',
            ],
            'joint_states_topic': '/joint_states',
            'odom_topic': '/mecanum_controller/odometry',
            'output_topic': '/mecanum_slip',
            'min_speed': 0.02,
            'publish_rate': 10.0,
            'vy_sign': 1.0,
            'wz_sign': 1.0,
            'use_sim_time': True,
        }],
    )

    odom_tf = Node(
        package='mecanum_bot',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/mecanum_controller/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': True,
        }],
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--param-file', controllers_yaml],
        output='screen'
    )

    mec = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager', '--param-file', controllers_yaml],
        output='screen'
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb, mec],
        )
    )

    return LaunchDescription([
        robot_file_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        lidar_tf,
        map_odom_tf,
        cmd_vel_scaler,
        slip_monitor,
        odom_tf,
        spawn_controllers,
    ])
