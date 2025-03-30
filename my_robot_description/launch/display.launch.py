import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition


from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package="my_robot_description").find("my_robot_description")
#    default_model_path = os.path.join(pkg_share, "src/description/my_robot_description.urdf")
    #xacro_file = os.path.join(pkg_share, "src/description/two_wheels.xacro")
    #robot_desc = xacro.process_file(xacro_file).toxml()
    xacro_file = os.path.join(pkg_share, 'src/description', 'my_robot_description.urdf')
    #with open(xacro_file, 'r') as infp:
    #    robot_desc = infp.read()
    robot_desc = xacro.process_file(xacro_file).toxml()

    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    """
    gui_arg = DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui"
            )
    """
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

 
    """
    model_arg = DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot urdf file"
            )
    """
    rvizconfig_arg = DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file"
            )
    """
    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ",
                                                    LaunchConfiguration("src/description")])}]
            )
    """
    joint_state_publisher_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            #condition=UnlessCondition(LaunchConfiguration("gui"))
            )
    """joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui"))
            )"""
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
            )

    world_file_name = 'world.sdf'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path], 
        'on_exit_shutdown': 'true'}.items()
        # launch_arguments={'world': world_path}.items(),
    )

  

    spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic','robot_description',
                       '-name', 'diffbot',
                       '-z', '1.0'],
            output='screen'
    )

    #ld.add_action(gui_arg)
    #ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(rvizconfig_arg)
    ld.add_action(joint_state_publisher_node)
    #ld.add_action(joint_state_publisher_gui_node)
    #ld.add_action(robot_state_publisher_node)
   # ld.add_action(rviz_node)


    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    return ld
