from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# controller = "bicycle_steering_controller"
controller = "ackermann_steering_controller"

def generate_launch_description():
    package_name = "ackerman_robot"
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "urdf", "ackermann.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "ackermann_controller.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", "carlikebot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/"+controller+"/reference", "/cmd_vel"),
        ],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

   # robot_controller_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    #)
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller,
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /'+controller+'/tf_odometry:=/tf',
                   ],
    )

    teleop_keyboard = Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            prefix="xterm -e",
            parameters=[{'stamped': True}],
            remappings=[('cmd_vel','/'+controller+'/reference')]
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        teleop_keyboard,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)