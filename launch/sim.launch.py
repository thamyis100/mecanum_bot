from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_bot')

    # alt_world = os.path.join(pkg_share, 'worlds', 'rtab_map_stage_world2.sdf')
    # default_world = os.path.join(pkg_share, 'worlds', 'mecanum_worlds.sdf')
    default_world = os.path.join(pkg_share, 'worlds', 'rtab_map_stage_world2.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_bot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    ros2_control_params = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_file_arg = DeclareLaunchArgument(
        'robot_file',
        default_value=xacro_file,
        description='URDF/Xacro file to load for robot_description'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='SDF world file to load in Gazebo'
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Spawn position X (meters)'
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Spawn position Y (meters)'
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.2',
        description='Spawn position Z (meters)'
    )
    use_ground_truth_tf_arg = DeclareLaunchArgument(
        'use_ground_truth_tf',
        default_value='true',
        description='Use Gazebo ground-truth pose for odom->base_link TF'
    )
    pose_bridge_arg = DeclareLaunchArgument(
        'pose_bridge',
        default_value='',
        description='ros_gz_bridge rule for Gazebo pose -> ROS Pose (auto if empty)'
    )
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='',
        description='Gazebo pose topic to use for ground-truth TF (auto if empty)'
    )
    pose_msg_type_arg = DeclareLaunchArgument(
        'pose_msg_type',
        default_value='pose_array',
        description='ROS message type for pose_tf_broadcaster (pose, pose_array, pose_stamped)'
    )
    scan_bridge_arg = DeclareLaunchArgument(
        'scan_bridge',
        default_value='/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        description='ros_gz_bridge rule for Gazebo LaserScan -> ROS LaserScan'
    )
    clock_bridge_arg = DeclareLaunchArgument(
        'clock_bridge',
        default_value='/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        description='ros_gz_bridge rule for Gazebo Clock -> ROS /clock'
    )

    robot_file = LaunchConfiguration('robot_file')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    use_ground_truth_tf = LaunchConfiguration('use_ground_truth_tf')
    pose_bridge = LaunchConfiguration('pose_bridge')
    pose_topic = LaunchConfiguration('pose_topic')
    pose_msg_type = LaunchConfiguration('pose_msg_type')
    scan_bridge = LaunchConfiguration('scan_bridge')
    clock_bridge = LaunchConfiguration('clock_bridge')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', robot_file,
                ' controllers_yaml:=', controllers_yaml,
                ' ros2_control_params:=', ros2_control_params
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
        launch_arguments={'gz_args': ['-r ', world]}.items(),
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
        arguments=[
            '-name', 'mecanum_bot',
            '-param', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
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

    scan_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        output='screen',
        arguments=[scan_bridge],
    )
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[clock_bridge],
    )

    cmd_vel_scaler = Node(
        package='mecanum_bot',
        executable='cmd_vel_scaler.py',
        name='cmd_vel_scaler',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/mecanum_controller/reference',
            'output_stamped': True,
            'scale_linear': 0.35,
            'scale_angular': 0.35,
            'max_input': 1.0,
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
        condition=UnlessCondition(use_ground_truth_tf),
        parameters=[{
            'odom_topic': '/mecanum_controller/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': True,
        }],
    )
    def _ground_truth_nodes(context, *args, **kwargs):
        if LaunchConfiguration('use_ground_truth_tf').perform(context).lower() != 'true':
            return []

        pose_topic_value = LaunchConfiguration('pose_topic').perform(context).strip() or '/model/mecanum_bot/pose'
        pose_bridge_value = LaunchConfiguration('pose_bridge').perform(context).strip()
        if not pose_bridge_value:
            pose_bridge_value = f'{pose_topic_value}@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
        pose_msg_type_value = LaunchConfiguration('pose_msg_type').perform(context).strip() or 'pose_array'

        pose_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_bridge',
            output='screen',
            arguments=[pose_bridge_value],
        )
        ground_truth_tf = Node(
            package='mecanum_bot',
            executable='pose_tf_broadcaster.py',
            name='pose_tf_broadcaster',
            output='screen',
            parameters=[{
                'pose_topic': pose_topic_value,
                'pose_msg_type': pose_msg_type_value,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'use_sim_time': True,
            }],
        )
        return [pose_bridge_node, ground_truth_tf]

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
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        use_ground_truth_tf_arg,
        pose_bridge_arg,
        pose_topic_arg,
        pose_msg_type_arg,
        scan_bridge_arg,
        clock_bridge_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        lidar_tf,
        map_odom_tf,
        scan_bridge_node,
        clock_bridge_node,
        cmd_vel_scaler,
        slip_monitor,
        odom_tf,
        OpaqueFunction(function=_ground_truth_nodes),
        spawn_controllers,
    ])
