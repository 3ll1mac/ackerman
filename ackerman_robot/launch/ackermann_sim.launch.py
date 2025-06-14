import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    package_name = "ackerman_robot"


    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('ackerman_robot'),
                    performed_description_format,
                    f'ackermann.urdf.xacro'
                ]),
            ]
        )

        pkg_path =  os.path.join(get_package_share_directory('ackerman_robot'))
        xacro_file = os.path.join(pkg_path, 'urdf', 'ackermann.urdf.xacro')

        robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
        robot_description = {'robot_description': robot_description_config}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ackerman_robot'),
            'config',
            'ackermann_controller.yaml',
        ]
    )

    world_file_name = 'new_world.sdf'
    pkg_share = get_package_share_directory('ackerman_robot')
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ackermann', '-allow_renaming', 'true'],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", "carlikebot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /ackermann_steering_controller/tf_odometry:=/tf',
                   ],
    )

    # Bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}',
        ]
    )

    # Teleop Twist Keyboard

    teleop_keyboard = Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            prefix="xterm -e",
            parameters=[{'stamped': True}],
            remappings=[('cmd_vel','/ackermann_steering_controller/reference')]
    )

    #  launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),

    ld = LaunchDescription([
        bridge,
        rviz_node,
        teleop_keyboard,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                launch_arguments=[('gz_args', ['-r ', world_path])]), 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
